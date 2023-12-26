/**
   @file peakLevelMeter.ino
   @author 
   @brief
   @version 0.1
   @date 2022-10-17

   @copyright Copyright (c) 2022

*/

#include <Arduino.h>
#include <math.h>
#include <driver/i2s.h>
#include <soc/io_mux_reg.h>

#define I2S_NUM_MIC       I2S_NUM_0
#define I2S_PIN_CLK_MIC   2
#define I2S_PIN_WS_MIC    4
#define I2S_PIN_DOUT_MIC  -1 //not use
#define I2S_PIN_DIN_MIC   15

#define I2S_SAMPLE_RATE   8000 //8kHz
#define I2S_BUFFER_COUNT  8
#define I2S_BUFFER_SIZE   512
#define Sound_LEN         512

uint8_t Mic_Buffer[I2S_BUFFER_SIZE];    // DMA Transfer buffer

struct data_t {
  union {
    char BYTE[8];
    uint8_t n[8];
    struct {
      int32_t r;
      int32_t l;
    };
  };
};

int32_t data_r[I2S_BUFFER_SIZE / I2S_BUFFER_COUNT];
int32_t data_l[I2S_BUFFER_SIZE / I2S_BUFFER_COUNT];


//
// dB calcurater
//
#define ZERODB 2147483648.0

float dbcalc(float x) {
  return 20.0 * log10(x / ZERODB);
}


//
// Envelope detecter
//
struct envelopeDetector_t {
  float envelope;
  float att;
} envelope1, envelope2;

void envelopeDetector_init(struct envelopeDetector_t *a, float att) {
  a->envelope = -100.0;
  a->att = att;
}

float envelopeDetector(struct envelopeDetector_t *a, float input) {
  float b;
  b = fabs(input);
  a->envelope = a->envelope * a->att;
  if (a->envelope < b) {
    a->envelope = b;
  }
  return a->envelope;
}

//
// Notch Filter
//
struct notchFilter_t {
  float bs_filter[2]; //stores previous filter outputs
  float previous_input[2];
  float r;
  float b0;
  float b1; //{Equation: -2.0*cos(2*PI*fc/float(fs));}
  float b2;
  float a1; //{Equation: 2*r*cos(2*PI*fc/float(fs));}
  float a2; //{Equation: -1.0 * pow(r,2);}
} filter1, filter2;

void notchFilter_init(struct notchFilter_t *a, float fc, float fs) {
  a->bs_filter[0] = 0; //stores previous filter outputs
  a->bs_filter[1] = 0; //stores previous filter outputs
  a->previous_input[0] = 0;
  a->previous_input[1] = 0;

  //Notch filter parameters & constants
  a->r = 0.8;
  a->b0 = 1;
  a->b1 = -2 * cos(2 * PI * fc / fs);
  a->b2 = 1;
  a->a1 = 2 * a->r * cos(2 * PI * fc / fs);
  a->a2 = -(a->r * a->r);
}

float notchFilter(struct notchFilter_t *a, float input) {
  float result;

  //calculate filter output
  result = input + (a->b1 * a->previous_input[1]) + (a->b2 * a->previous_input[0]) + (a->a1 * a->bs_filter[1]) + (a->a2 * a->bs_filter[0]);

  //update filter output values
  a->bs_filter[0] = a->bs_filter[1];
  a->bs_filter[1] = result;
  a->previous_input[0] = a->previous_input[1];
  a->previous_input[1] = input;

  //update output array
  return result;
}


//
//  I2S Peak level meter
//
void i2s_peak() {
  size_t transBytes;
  struct data_t a;
  int count = 0;

  i2s_read(I2S_NUM_MIC, (char*)Mic_Buffer, I2S_BUFFER_SIZE, &transBytes, portMAX_DELAY);  // piezo in

  // Convert 8 bytes data to L_Data and R_Data
  for (int i = 0; i < transBytes; i = i + 8) {
    for (int j = 0; j < 8; j++) {
      a.BYTE[j] = Mic_Buffer[i + j];
    }
    data_r[count] = envelopeDetector(&envelope1, notchFilter(&filter1, a.r));
    data_l[count] = envelopeDetector(&envelope2, notchFilter(&filter2, a.l));
    count ++;
  }

  //
  // Output peak level
  //
  Serial.print(dbcalc(data_r[63]));   Serial.print(",");  Serial.println(dbcalc(data_l[63]));
  Serial.flush();
}

void i2s_init() {
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = I2S_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT, //stereo
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = I2S_BUFFER_COUNT,
    .dma_buf_len          = I2S_BUFFER_SIZE,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num           = I2S_PIN_CLK_MIC,
    .ws_io_num            = I2S_PIN_WS_MIC,
    .data_out_num         = I2S_PIN_DOUT_MIC,
    .data_in_num          = I2S_PIN_DIN_MIC,
  };

  i2s_driver_install(I2S_NUM_MIC, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_MIC, &pin_config);
  
  // CLK_OUT1 ->  GPIO0  ->  MCLK pin on AK5720VT
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL) & 0xFFFFFFF0);
}


//
// calucurate attenuation value from attenuation ratio
//

float attValueCalc(float attenuation) {
  return pow(pow(10.0, attenuation / 20.0), (1.0 / I2S_SAMPLE_RATE));
}


//////////////////
// Measure task //
//////////////////

TaskHandle_t measureHandle = NULL;

void measureStart(void) {
  xTaskCreateUniversal(measureTask, "measureTask", 8192, NULL, 3, &measureHandle, APP_CPU_NUM);
}

void measureTask(void *pvParameters) {
  while (true) {
    i2s_peak();
  }
}

///////////
// Setup //
///////////

#define ATTENUATION1 -20.0  // -10dB/sec Peak hold
#define ATTENUATION2 -160.0  // -80dB/sec Level meter

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(50);

  notchFilter_init(&filter1, 30, I2S_SAMPLE_RATE);
  notchFilter_init(&filter2, 30, I2S_SAMPLE_RATE);

  envelopeDetector_init(&envelope1, attValueCalc(ATTENUATION2));
  envelopeDetector_init(&envelope2, attValueCalc(ATTENUATION2));

  i2s_init();

  measureStart();
}

//////////
// Loop //
//////////

void loop() {}

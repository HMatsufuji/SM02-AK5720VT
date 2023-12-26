/**
   @file simpleWaveform.ino
   @author 
   @brief
   @version 0.1
   @date 2022-10-17

   @copyright Copyright (c) 2022

*/

#include <Arduino.h>
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

//
//  I2S waveform loader
//
void i2s_peak() {
  size_t transBytes;
  struct data_t a;

  i2s_read(I2S_NUM_MIC, (char*)Mic_Buffer, I2S_BUFFER_SIZE, &transBytes, portMAX_DELAY);

  // Convert 8 bytes data to L_Data and R_Data
  for (int i = 0; i < transBytes; i = i + 8) {
    for (int j = 0; j < 8; j++) {
      a.BYTE[j] = Mic_Buffer[i + j];
    }
    //
    // Output waveform
    // The waveform overflows because the serial plot is slower relative to the sampling period.
    //
    Serial.print(a.r);   Serial.print(",");  Serial.println(a.l);
    Serial.flush();
  }

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

void setup() {

  Serial.begin(115200);
  while (!Serial) delay(50);

  i2s_init();

  measureStart();
}

//////////
// Loop //
//////////

void loop() {}

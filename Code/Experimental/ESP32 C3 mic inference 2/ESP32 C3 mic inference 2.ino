#include <ollieelliot-project-1_inferencing.h>
#include <Arduino.h>
#include <driver/i2s.h>


/*************** I2C Macros ***************/
// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 41100
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK 6
#define I2S_MIC_LEFT_RIGHT_CLOCK 5
#define I2S_MIC_SERIAL_DATA 4

/*************** Other Macros ***************/
#define LED 0

/*************** Global variables ***************/
uint8_t ret = -1;
int16_t raw_samples[SAMPLE_BUFFER_SIZE];
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
int indx = 0;

// don't mess around with this
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};


void setup()
{
  // we need serial output for the plotter
  Serial.begin(115200);
  // start up the I2S peripheral
  ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if(ret) Serial.printf("[Error] %d",ret);
  ret = i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  if(ret) Serial.printf("[Error] %d",ret);

  pinMode(LED,OUTPUT);
  
}

void loop()
{
  // read from the I2S device
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int16_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int16_t);
  
  for(int i=0; i<samples_read; i++)
  {
    features[i+indx] = float(raw_samples[i]);
  }
  indx += samples_read;

  if(indx >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
  {
    // start inferencing
    ei_impulse_result_t result;

    // create signal from features frame
    signal_t signal;
    numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    // run classifier
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    //ei_printf("run_classifier returned: %d\n", res);
    if (res != 0)
      return;

    //****************
    // print predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    // print the predictions
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
      ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    //*****************/

   if(result.classification[1].value>0.75) //Label Cry
    {
    digitalWrite(LED,HIGH);
    }

   else
    {
    digitalWrite(LED,LOW);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("anomaly:\t%.3f\n", result.anomaly);
#endif

    indx = 0;
  }
}
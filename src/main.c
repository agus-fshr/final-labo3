/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/********************** inclusions *******************************************/
/* Project includes. */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/dac.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "ADC_continuous.h"
#include <math.h>

/********************** macros and definitions *******************************/
#define UPDATE_PERIOD_MICROSECONDS 100  //[T]=usec
#define UPDATE_SAMPLE_FRECUENCY 1000000.0/UPDATE_PERIOD_MICROSECONDS  //[Fs]= Hz
#define N  (int) ((UPDATE_SAMPLE_FRECUENCY)/DOn1)


#define DAC_CHANNEL DAC_CHAN_0

// Echo defines
#define DELAY_BUFFER_SIZE 9000 * 2

// Tremolo defines
#define SAMPLE_RATE 9000
#define TREMOLO_FREQUENCY 5
#define TREMOLO_INCREMENT 2 * M_PI * TREMOLO_FREQUENCY / SAMPLE_RATE

/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
	uint32_t i =0;
/********************** external data declaration *****************************/


/********************** external functions definition ************************/

// Echo
static uint32_t delay_buffer[DELAY_BUFFER_SIZE] = {0};
static int delay_index = 0;

// Tremolo
float tremolo_phase = 0.0;


void app_main(void)
{
    //GPIO Initialize
    gpio_config_t io_config;
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.intr_type = GPIO_INTR_DISABLE;
    io_config.pin_bit_mask = (1ULL << GPIO_NUM_32)|(1ULL << GPIO_NUM_33);//|(1ULL << GPIO_NUM_35);
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 0;
    gpio_config(&io_config);

    //ADC initialize
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
//    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));


    //DAC Initialize
    dac_output_enable(DAC_CHANNEL);

    // Variables para mantener el tiempo de actualización
    uint64_t current_time = 0;
    int i =0;


    while (1) 
    {
        gpio_set_level(GPIO_NUM_32, 1);
        ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
        if (ret == ESP_OK) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
            uint32_t data = EXAMPLE_ADC_GET_DATA(p);
            delay_buffer[delay_index] = data;
            delay_index = (delay_index + 1) % DELAY_BUFFER_SIZE;
            uint32_t data_out = data;
            
            //  Process audio data
            
            // Echo
            // data_out = (uint32_t)(data + (uint32_t)(delay_buffer[delay_index-1]))/2;

            // Distortion
            data_out *= 2;

            // Tremolo
            // float lfo_value = (sin(tremolo_phase) + 1.0) / 2.1 + 0.1;
            // data_out *= lfo_value;
            // tremolo_phase += TREMOLO_INCREMENT;
            // if (tremolo_phase >= 2 * M_PI) {
            //     tremolo_phase -= 2 * M_PI;
            // }



            gpio_set_level(GPIO_NUM_33, 1);
            gpio_set_level(GPIO_NUM_33, 0);
            while ((esp_timer_get_time() - current_time) < (UPDATE_PERIOD_MICROSECONDS));
            dac_output_voltage(DAC_CHANNEL, (int) (data_out >>4 & 0x000000FF));
            // int scaled_value = data_out / 1.5;
            // dac_output_voltage(DAC_CHANNEL, scaled_value);
            current_time = esp_timer_get_time(); // Obtiene el tiempo actual en microsegundos
        } else if (ret == ESP_ERR_TIMEOUT) {
            //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            break;
        }
        gpio_set_level(GPIO_NUM_32, 0);
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
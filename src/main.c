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
#define UPDATE_PERIOD_MICROSECONDS 50  //[T]=usec
#define UPDATE_SAMPLE_FRECUENCY 1000000.0/UPDATE_PERIOD_MICROSECONDS  //[Fs]= Hz
#define N  (int) ((UPDATE_SAMPLE_FRECUENCY)/DOn1)


#define DAC_CHANNEL DAC_CHAN_0
#define REVERB_BUTTON GPIO_NUM_16
#define ECHO_BUTTON GPIO_NUM_17
#define DISTORTION_BUTTON GPIO_NUM_18
#define TREMOLO_BUTTON GPIO_NUM_19
#define RECTIFIED_TREMOLO GPIO_NUM_22
#define CLEAN_BUTTON GPIO_NUM_23

// Echo defines
#define DELAY_BUFFER_SIZE 20000
#define FIXED_DECIMAL_ONE 1000
#define FIXED_DECIMAL(x) ( (int)((x) *  FIXED_DECIMAL_ONE) )
#define MIX(v1, v2, beta) ( ((v1) * beta + (FIXED_DECIMAL_ONE - beta) * (v2)) / FIXED_DECIMAL_ONE)
#define ECHO_FEEDBACK  FIXED_DECIMAL(0.5)
#define HISTORY_LENGTH (1<<14)
#define HISTORY_MASK (HISTORY_LENGTH - 1)

// Tremolo defines
#define SAMPLE_RATE 22000
#define TREMOLO_FREQUENCY 5
#define TREMOLO_INCREMENT 2 * M_PI * TREMOLO_FREQUENCY / SAMPLE_RATE



/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
	uint32_t i =0;
/********************** external data declaration *****************************/


/********************** external functions definition ************************/

// Echo
static uint32_t echo_buffer[DELAY_BUFFER_SIZE] = {0};
static uint32_t past_samples[DELAY_BUFFER_SIZE] = {0};
static int delay_index = 0;

// Tremolo
float tremolo_phase = 0.0;

void configure_gpio_pins(void) {
    // Create a bitmask for all the buttons
    uint64_t button_mask = (1ULL << REVERB_BUTTON) |
                           (1ULL << ECHO_BUTTON) |
                           (1ULL << DISTORTION_BUTTON) |
                           (1ULL << TREMOLO_BUTTON) |
                           (1ULL << RECTIFIED_TREMOLO);

    // Configure GPIOs as input with pull-up resistors
    gpio_config_t io_conf = {
        .pin_bit_mask = button_mask,      // Select the buttons
        .mode = GPIO_MODE_INPUT,          // Set as input mode
        .pull_up_en = GPIO_PULLUP_ENABLE, // Enable pull-up resistors
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistors
        .intr_type = GPIO_INTR_DISABLE    // Disable interrupts for now
    };
    gpio_config(&io_conf);
}

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
    configure_gpio_pins();

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
        gpio_set_level(GPIO_NUM_32, 0);
        if (ret == ESP_OK) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
            uint32_t data = EXAMPLE_ADC_GET_DATA(p);
            // echo_buffer[delay_index] = data;
            past_samples[delay_index] = data;
            delay_index = (delay_index + 1) % DELAY_BUFFER_SIZE;
            uint32_t data_out = data;
            uint32_t prev_index = 0;

            //  Process audio data
            uint8_t effect_selected = 3;
            // Reverb
            if (gpio_get_level(REVERB_BUTTON) == 0) {
                prev_index = (delay_index == DELAY_BUFFER_SIZE) ? (0) : (delay_index + 1);
                echo_buffer[delay_index] = (uint32_t)(data + (uint32_t)(0.75 * echo_buffer[prev_index]));
                data_out = echo_buffer[delay_index]/2;
            }
            
            if (gpio_get_level(ECHO_BUTTON) == 0) {
                prev_index = (delay_index == DELAY_BUFFER_SIZE / 2) ? (0) : (delay_index + 1);
                echo_buffer[delay_index] = (uint32_t)(past_samples[delay_index-1] + (uint32_t)(0.75 * echo_buffer[prev_index]));
                data_out = echo_buffer[delay_index]/2;
            }

            if (gpio_get_level(DISTORTION_BUTTON) == 0) {
                data_out *= 2;
            }

            if (gpio_get_level(TREMOLO_BUTTON) == 0) {
                float lfo_value = (sin(tremolo_phase) + 1.0) / 2.1 + 0.1;
                data_out *= lfo_value;
                tremolo_phase += TREMOLO_INCREMENT;
                if (tremolo_phase >= 2 * M_PI) {
                    tremolo_phase -= 2 * M_PI;
                }
            }

            if (gpio_get_level(RECTIFIED_TREMOLO) == 0) {
                if (data_out < 1000) {
                    data_out = 1000 + (1000 - data_out);
                } 
            }
            // Downsample
            // if (delay_index % 2 == 0) {
            //     data_out = past_samples[delay_index - 1];
            // }




            gpio_set_level(GPIO_NUM_33, 1);
            gpio_set_level(GPIO_NUM_33, 0);
            while ((esp_timer_get_time() - current_time) < (UPDATE_PERIOD_MICROSECONDS));
            // vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MICROSECONDS / 1000));
            dac_output_voltage(DAC_CHANNEL, (int) (data_out >>4 & 0x000000FF));
            // int scaled_value = data_out / 1.5;
            // dac_output_voltage(DAC_CHANNEL, scaled_value);
            current_time = esp_timer_get_time(); // Obtiene el tiempo actual en microsegundos
        } else if (ret == ESP_ERR_TIMEOUT) {
            //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            break;
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
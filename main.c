#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include <math.h>
#include "driver/ledc.h"

#define DIG1 26
#define DIG2 27
#define DIG3 14

#define A 15
#define B 19
#define C 16
#define D 2
#define E 4
#define F 17
#define G 23

#define LED_RED 13
#define LED_GREEN 12

#define Left_button 21
#define right_button 32
#define Left 33
#define right 22

#define potenciometro 34
#define SAMPLE_PERIOD_US 20000

// Variables globales compartidas (ISR + main)
static volatile int value1 = 0;
static volatile int value2 = 0;
static volatile int value3 = 0;
static volatile int direccion = 0;

// Antirrebote por tiempo
static volatile int64_t antireizq = 0;
static volatile int64_t antireder = 0;

// Tabla de segmentos para display 7 segmentos
const uint8_t digitos[10] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
    0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111
};

// Activa segmentos según patrón
void mostrar_segmentos(uint8_t patron) {
    gpio_set_level(A, !((patron >> 0) & 1));
    gpio_set_level(B, !((patron >> 1) & 1));
    gpio_set_level(C, !((patron >> 2) & 1));
    gpio_set_level(D, !((patron >> 3) & 1));
    gpio_set_level(E, !((patron >> 4) & 1));
    gpio_set_level(F, !((patron >> 5) & 1));
    gpio_set_level(G, !((patron >> 6) & 1));
}

// Multiplexado de los 3 displays
void multiplexar_display(int num1, int num2, int num3) {
    gpio_set_level(DIG1, 1);
    gpio_set_level(DIG2, 1);
    gpio_set_level(DIG3, 1);
    mostrar_segmentos(digitos[num1]);
    gpio_set_level(DIG1, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(DIG1, 1);
    gpio_set_level(DIG2, 1);
    gpio_set_level(DIG3, 1);
    mostrar_segmentos(digitos[num2]);
    gpio_set_level(DIG2, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_level(DIG1, 1);
    gpio_set_level(DIG2, 1);
    gpio_set_level(DIG3, 1);
    mostrar_segmentos(digitos[num3]);
    gpio_set_level(DIG3, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
}

// Interrupción botón izquierda (con antirrebote)
static void IRAM_ATTR izquierda(void *arg){
    int64_t now = esp_timer_get_time();
    if (now - antireizq > 200000){
        direccion = 1;
        antireizq = now;
    }
}

// Interrupción botón derecha (con antirrebote)
static void IRAM_ATTR derecha(void *arg){
    int64_t now = esp_timer_get_time();
    if (now - antireder > 200000){
        direccion = 2;
        antireder = now;
    }
}

void app_main() {

    // Estado inicial del sistema
    gpio_set_level(LED_RED, 1);
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(Left, 1);
    gpio_set_level(right, 0);
    direccion = 0;

    // Configuración de salidas (display, LEDs, control)
    gpio_config_t leds = {
        .pin_bit_mask = (
            (1ULL << A) | (1ULL << B) | (1ULL << C) |
            (1ULL << D) | (1ULL << E) | (1ULL << F) |
            (1ULL << G) | (1ULL << DIG1) | (1ULL << DIG2) |
            (1ULL << DIG3) | (1ULL << LED_RED) |
            (1ULL << LED_GREEN) | (1ULL << Left) | (1ULL << right)
        ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&leds);

    // Configuración de botones con interrupciones
    gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << Left_button | 1ULL << right_button),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&inputs);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(Left_button, izquierda, NULL);
    gpio_isr_handler_add(right_button, derecha, NULL);

    // Timer para muestreo periódico del ADC
    timer_config_t timer_pro = {
        .divider = 80,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_pro);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    uint64_t timer_value = 0;
    int adc_raw;

    // Configuración ADC (lectura del potenciómetro)
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config);

    // Configuración PWM (control de potencia)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = 25,
        .duty = 0,
    };
    ledc_channel_config(&ledc_channel);

    while(1){
        multiplexar_display(value1, value2, value3);

        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_value);

        // Muestreo periódico del ADC + actualización de display y PWM
        if (timer_value >= SAMPLE_PERIOD_US) {

            adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw);

            int porcentaje = (adc_raw * 100) / 4095;

            value1 = porcentaje / 100;
            value2 = (porcentaje / 10) % 10;
            value3 = porcentaje % 10;

            timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

            int duty = (porcentaje * 4095) / 100;

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }

        // Cambio de dirección mediante botones
        if (direccion == 1){
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(500));

            gpio_set_level(LED_RED, 1);
            gpio_set_level(LED_GREEN, 0);
            gpio_set_level(Left, 1);
            gpio_set_level(right, 0);
            direccion = 0;

        } else if (direccion == 2) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(500));

            gpio_set_level(LED_RED, 0);
            gpio_set_level(LED_GREEN, 1);
            gpio_set_level(Left, 0);
            gpio_set_level(right, 1);
            direccion = 0;
        }
    }
}

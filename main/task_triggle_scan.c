#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#define TRIGGER_PIN 32        //触发信号

int triggle_flag = 0;

static void IRAM_ATTR intrHandler (void *arg){
    triggle_flag = triggle_flag +1 ;
    
}


void server_triggle(){
    gpio_config_t gpioTRIGGER_PIN = {
        .pin_bit_mask = 1ULL << TRIGGER_PIN,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&gpioTRIGGER_PIN);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(TRIGGER_PIN, intrHandler, (void*)TRIGGER_PIN);
    printf("gpio_isr_handler_add\n");


}
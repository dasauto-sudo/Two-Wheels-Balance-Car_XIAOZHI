#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "hardware_init.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_timer.h"

// 延时函数(毫秒)
void delay_ms(uint32_t ms) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (ms * 1000ULL)) {
        // 等待指定毫秒数
    }
}

extern "C" void app_main(void)
{
    // // 初始化I2C
    // init_i2c();
    // ESP_LOGI("MAIN", "I2C initialized");

    // spi_slave_init();
    
    // // 初始化传感器
    // init_sensors();
    // ESP_LOGI("MAIN", "Sensors initialized");
    
    // // 初始化电机
    // init_motors();
    // ESP_LOGI("MAIN", "Motors initialized");
    
    // // 初始化IMU
    // init_imu();
    // ESP_LOGI("MAIN", "IMU initialized");

    hardware_init();

    ESP_LOGI("MAIN", "Balance control system started");
    
    // 主循环
    while (1) {
        
        
        // 平衡控制逻辑 (100Hz)
        balance_control_loop();
        
        // 延时10ms，保持100Hz控制频率
        delay_ms(10);
    }
}
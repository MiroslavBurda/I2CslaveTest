#include <esp_log.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <driver/i2c.h>

#define RETURN_IF_ERR(x) do {                                         \
        esp_err_t __err_rc = (x);                                       \
        if (__err_rc != ESP_OK) {                                       \
            return __err_rc;                                            \
        }                                                               \
    } while(0)

gpio_num_t sda_pin = GPIO_NUM_18;
gpio_num_t scl_pin = GPIO_NUM_19; 
// uint32_t speed_hz = 100000;
uint8_t address = 0x15;
i2c_port_t bus_num = I2C_NUM_0; 
uint8_t DataToReceive[] = {2, 2, 2, 2};
size_t len = sizeof(DataToReceive);
uint8_t DataToSend[] = {11, 12, 13, 14, 21, 22, 23, 24};

i2c_config_t conf_slave = {
    .mode = I2C_MODE_SLAVE,
    .sda_io_num = sda_pin,
    .scl_io_num = scl_pin,            
    .sda_pullup_en = GPIO_PULLUP_ENABLE,  
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .slave = {
        .addr_10bit_en = 0,
        .slave_addr = address,  
    },
    .clk_flags = 0, 
};

int i = 0; 

extern "C" void app_main() {  // example for connect ESP32 with I2C
    ESP_ERROR_CHECK(i2c_param_config(bus_num, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(bus_num, I2C_MODE_SLAVE, 2000, 2000, 0)); 
    i2c_slave_write_buffer(bus_num, DataToSend, 8, pdMS_TO_TICKS(25)); 

    while (true) {
        i2c_slave_read_buffer(bus_num, DataToReceive, 4, pdMS_TO_TICKS(25)); 
        for (int k = 0; k < 4; k++) {
            ESP_LOGI("DATA:", "%i \n", DataToReceive[k] );
        }
        ESP_LOGI("TAG1", "%i \n", i++);  
        vTaskDelay(pdMS_TO_TICKS(1000));   
    }
}

#include <esp_log.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <driver/gpio.h>
#include "time.hpp"
#include "stopwatch.hpp"
#include <driver/i2c.h>

#define RETURN_IF_ERR(x) do {                                         \
        esp_err_t __err_rc = (x);                                       \
        if (__err_rc != ESP_OK) {                                       \
            return __err_rc;                                            \
        }                                                               \
    } while(0)

#define GPIO_OUTPUT_PIN_SEL ( 1ULL << GPIO_NUM_17 | 1ULL <<  GPIO_NUM_22 )
void iopins_init(void) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

gpio_num_t sda_pin = GPIO_NUM_32;
gpio_num_t scl_pin = GPIO_NUM_33; 
uint32_t speed_hz = 100000;
uint8_t address = 0x15;
i2c_port_t bus_num = I2C_NUM_0; 
uint8_t DataToReceive[] = {2, 2, 2, 2};
size_t len = sizeof(DataToReceive);


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

// esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num,
//                      bool sda_pullup_en, bool scl_pullup_en, i2c_mode_t mode);

timeout send_data { msec(1000) }; // timeout zajistuje posilani dat do PC kazdych 1000 ms
bool L_G = true;
int i = 0; 

extern "C" void app_main() {  // example for connect ESP32 with I2C
    iopins_init();
    gpio_set_level(GPIO_NUM_17, 1);
    ESP_ERROR_CHECK(i2c_param_config(bus_num, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(bus_num, I2C_MODE_SLAVE, 2000, 2000, 0)); 
    gpio_set_level(GPIO_NUM_22, 1); // yellow diode = ready for accepting data 

    while (true) {
        ESP_ERROR_CHECK(i2c_slave_read_buffer(bus_num, DataToReceive, 4, pdMS_TO_TICKS(25))); 
        for (int k = 0; k < 4; k++) {
            ESP_LOGI("DATA:", "%i, ", DataToReceive[k] );
        }

        //if (send_data) {  // is board still working? 
        //    send_data.ack();
            if (L_G) L_G = false; else  L_G = true;
            gpio_set_level(GPIO_NUM_17, L_G);
        //}

        ESP_LOGI("TAG1: ", "%i \n", i++);  
        vTaskDelay(pdMS_TO_TICKS(1000));   
    }
}

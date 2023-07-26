//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "JRC_WiFi.h"

#include "mbcontroller.h"

#include "esp_log.h"
#include "esp_err.h"

//____________________________________________________________________________________________________
// Macro definitions:
//____________________________________________________________________________________________________

#define MB_REG_INPUT_START_AREA0    (0)
#define MB_REG_HOLDING_START_AREA0  (0)
#define MB_REG_HOLD_CNT             (100)
#define MB_REG_INPUT_CNT            (100)

//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________
static const char *TAG = "SLAVE_TEST";

esp_netif_t* esp_netif_ptr = NULL;

mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
uint16_t holding_reg_area[MB_REG_HOLD_CNT] = {0}; // storage area for holding registers
uint16_t input_reg_area[MB_REG_INPUT_CNT] = {0}; // storage area for input registers

// Statically allocate and initialize the spinlock
static portMUX_TYPE mb_spinlock = portMUX_INITIALIZER_UNLOCKED;

//____________________________________________________________________________________________________
// Function prototypes:
//____________________________________________________________________________________________________
esp_err_t modbus_slave_init(void);

//____________________________________________________________________________________________________
// Main program:
//____________________________________________________________________________________________________
void app_main(void)
{
    JRC_WiFi_Begin();

    modbus_slave_init();

    while (1){

        taskENTER_CRITICAL(&mb_spinlock);
        for (int i =0; i<MB_REG_HOLD_CNT; i++){
            holding_reg_area[i]++;
            input_reg_area[i]+=5;
        }
        taskEXIT_CRITICAL(&mb_spinlock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//____________________________________________________________________________________________________
// Function implementations:
//____________________________________________________________________________________________________

esp_err_t modbus_slave_init(void){

    // Stage 1. Modbus Port Initialization:

    void* slave_handler = NULL; // Pointer to allocate interface structure
    // Initialization of Modbus slave for TCP
    esp_err_t err = mbc_slave_init_tcp(&slave_handler);
    if (slave_handler == NULL || err != ESP_OK) {
        // Error handling is performed here
        ESP_LOGE(TAG, "mb controller initialization fail.");
    }

    //Stage 2. Configuring Slave Data Access:

    reg_area.type = MB_PARAM_HOLDING;                               // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0;             // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_area[0];                 // Set pointer to storage instance
    reg_area.size = sizeof(holding_reg_area) << 1;                  // Set the size of register storage area in bytes
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;
    reg_area.address = (void*)&input_reg_area[0];
    reg_area.size = sizeof(input_reg_area) << 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Stage 3. Slave Communication Options:

    mb_communication_info_t comm_info = {
        .ip_port = 502,                            // Modbus TCP port number (default = 502)
        .ip_addr_type = MB_IPV4,                   // version of IP protocol
        .ip_mode = MB_MODE_TCP,                    // Port communication mode
        .ip_addr = NULL,                           // This field keeps the client IP address to bind, NULL - bind to any client
        .ip_netif_ptr = esp_netif_ptr              // esp_netif_ptr - pointer to the corresponding network interface
    };

    // Setup communication parameters and start stack
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    //Stage 4. Slave Communication Start:

    ESP_ERROR_CHECK(mbc_slave_start());

    return ESP_OK;
}
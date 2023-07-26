/*
______________________________________________________________________________________                                
                                JRC_WiFi Library V1.0
                                by:  Javier Ruzzante C
                                   (March 2023)
______________________________________________________________________________________
*/
 
#ifndef _JRC_WIFI
#define _JRC_WIFI

/**
 * @brief Define this macro for WiFi storage of data in RAM, if not defined (comment definition),
 *        the storage will be done in NVS by default.
 * 
 * @note  If NVS is not desired for WiFi, then disable NVS for WiFi via menuconfig.
*/
#define JRC_WIFI_USE_RAM_STORAGE
 
#define JRC_WIFI_NOT_STARTED    0
#define JRC_WIFI_CONNECTED      1
#define JRC_WIFI_STARTED        2
#define JRC_WIFI_GOT_IP         3
#define JRC_WIFI_DISCONNECTED   4
#define JRC_WIFI_AP_STARTED     5
 
#define DEFAULT_SCAN_LIST_SIZE 10
#define MAC_ADDR_LENGTH 18
#define IP_ADDR_LENGTH 16
 
typedef int JRC_WiFi_Status_t;
 
#include "esp_err.h"
#include "esp_wifi.h"

extern esp_netif_t* esp_netif_ptr;


/**
 * @brief These functions perfom the task needed to establish connection to the AP : 
 * 
 * The JRC_WiFi_Begin(); sets up the device in Station Mode,
 * The JRC_WiFi_Begin_STA_AP(); sets up the device in Station + SoftAP Mode.
 * 
 * @note The SSID and PASSWORD for the Station Mode and for the SoftAP Mode,
 *       are configured in the file secrets.h, under the corresponding macros.
 * 
 * @note If the macros `JRC_WIFI_STA_CUSTOM_MAC` and/or `JRC_WIFI_AP_CUSTOM_MAC` are defined,
 *       custom mac addresses will be used, if these macros are not defined, the factory mac
 *       addresses will be used instead.
 *       Edit the secrets.h file for the desired custom macs, or comment the macro definitions
 *       if custom macs are not desired.
 * 
 * @return `ESP_OK`
 */
esp_err_t         JRC_WiFi_Begin();
esp_err_t         JRC_WiFi_Begin_STA_AP();
// ______________________________________________________________________________________   
 

 /**
 * @brief Initialize the WiFi device as an AP in SoftAP Mode: 
 *
 * @note The SSID and PASSWORD for the Station Mode and for the SoftAP Mode,
 *       are configured in the file secrets.h, under the corresponding macros.
 * 
 * @note If the macro `JRC_WIFI_AP_CUSTOM_MAC` is defined, custom mac address will be used, 
 *       if this macro is not defined, the factory AP mac address will be used instead.
 *       Edit the secrets.h file for the desired custom mac, or comment the macro definition
 *       if custom mac is not desired.
 *
 * @return `ESP_OK`
 */
esp_err_t         JRC_WiFi_Begin_AP();
// ______________________________________________________________________________________   
 
 

/**
 * @brief Disconnect from AP in Station Mode, stops WiFi driver and free resources: 
 *
 * @return `ESP_OK` on success
 */
esp_err_t         JRC_WiFi_Stop();
// ______________________________________________________________________________________   
 
 
/**
 * @brief Get the status of the WiFi: The status is a typedef, that is coded in macros,
 * and can be one of the following: 
 *
 *       `JRC_WIFI_NOT_STARTED`
 *       `JRC_WIFI_CONNECTED` 
 *       `JRC_WIFI_STARTED` 
 *       `JRC_WIFI_GOT_IP` 
 *       `JRC_WIFI_DISCONNECTED` 
 * 
 * @return `ESP_OK`
 */ 
JRC_WiFi_Status_t JRC_WiFi_Get_Status();
// ______________________________________________________________________________________   
 
 
 
/**
 * @brief Get the information of the AP the device has been connected:
 * 
 * It works printing ap's info in the serial console (pass `NULL` as parameter), or saving the ap's info
 * in the structure passed as a parameter. 
 *
 * Call of function requires a pointer to a `wifi_ap_record_t` struct. ie: `JRC_WiFi_AP_Info(&ap_info);`
 *
 * @param apinfo Pointer to a struct declared as `wifi_ap_record_t`. ie: `wifi_ap_record_t ap_info;`
 *               If this parameter is `NULL`, the output result will be printed in the serial console.
 * 
 * @return `ESP_OK`
 */ 
esp_err_t         JRC_WiFi_AP_Info(wifi_ap_record_t *apinfo);
// ______________________________________________________________________________________   
 
 

/**
 * @brief Get the MAC Address of the ESP32 in Station Mode : It works printing MAC address in the
 * serial console (pass `NULL` as parameter), or saving the MAC address in a char array passed as a parameter. 
 *
 * Call of function requires a char array name, whose size must be 18 bytes at minimum.
 * Use the macro: `MAC_ADDR_LENGTH` to declare the char array.
 * 
 * ie:      `char macAddress[MAC_ADDR_LENGTH];`
 *          `JRC_WiFi_STA_MAC_Address(macAddress);`
 * 
 *
 * @param mac  Name of the previous declared char array. If this parameter is `NULL`, the MAC address will be
 * printed in the serial console.
 * 
 * @return `ESP_OK`
 */
esp_err_t         JRC_WiFi_STA_MAC_Address(char *mac);
// ______________________________________________________________________________________   
 
 
 

/**
 * @brief Get the IP address assigned to the ESP32 by the AP : Prints the IP address to the console,
 * (pass `NULL` as parameter), or saving the IP address in a char array passed as a parameter. 
 *
 * Call of function requires a char array name, whose size must be 16 bytes at minimum.
 * Use the macro: `IP_ADDR_LENGTH` to declare the char array.
 * 
 * ie:      `char ipAdd[IP_ADDR_LENGTH];`
 *          `JRC_WiFi_STA_IP_Address(ipAdd);`
 *
 * @param ipAdd  Name of the previous declared char array. If this parameter is `NULL`, the IP address will be
 * printed in the serial console.
 * 
 * @return `ESP_OK`
 */ 
esp_err_t         JRC_WiFi_STA_IP_Address(char *ipAdd);
// ______________________________________________________________________________________   
 
 
 

/**
 * @brief Get the Subnet Mask in Station Mode : Prints the Subnet Mask to the console, (pass `NULL` as parameter),
 * or saving the Subnet Mask in a char array passed as a parameter.
 *
 * Call of function requires a char array name, whose size must be 16 bytes at minimum. 
 * Use the macro: `IP_ADDR_LENGTH` to declare the char array.
 * 
 * ie:         `char snMask[IP_ADDR_LENGTH];`
 *             `JRC_WiFi_STA_Subnet_Mask(snMask);`

 *
 * @param snMask  Name of the previous declared char array. If this parameter is `NULL`, the Subnet Mask will be 
 * printed in the serial console.
 * 
 * @return `ESP_OK`
 */
esp_err_t         JRC_WiFi_STA_Subnet_Mask(char *snMask);
// ______________________________________________________________________________________   
 
 
 
/**
 * @brief Get the Gateway in Station Mode : Prints the Gateway to the console, (pass `NULL` as parameter),
 * or saving the Gateway in a char array passed as a parameter. 
 *
 * Call of function requires a char array name, whose size must be 16 bytes at minimum. 
 * Use the macro: `IP_ADDR_LENGTH` to declare the char array.
 * 
 * ie:         `char gateWay[IP_ADDR_LENGTH];` 
 * JRC_        `WiFi_STA_Gateway(gateWay);`
 *
 * @param gateWay  Name of the previous declared char array. If this parameter is `NULL`, the Gateway will be 
 * printed in the serial console.
 * 
 * @return `ESP_OK`
 */
esp_err_t         JRC_WiFi_STA_Gateway(char *gateWay);
// ______________________________________________________________________________________   
 


/**
 * @brief Perform a scan for available AP's: It works printing ap's info in the serial console (pass `NULL` as parameter),
 * or saving the ap's info in the array passed as a parameter
 *
 * 
 *
 * @param ap_found_info  array name declared as `wifi_ap_record_t`.
 * 
 * ie:      `wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];`
 *          `JRC_WiFi_Scan(ap_info);`
 * 
 * If this parameter is `NULL`, the output result will be printed in the serial console.
 * 
 * @return `ESP_OK`
 */ 
esp_err_t         JRC_WiFi_Scan(void *ap_found_info);
// ______________________________________________________________________________________   
 
//#include  "JRC_WiFi.c"
#endif

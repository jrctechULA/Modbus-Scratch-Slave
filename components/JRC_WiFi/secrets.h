/*
______________________________________________________________________________________                                
                                JRC_WiFi Library V1.0
                                by:  Javier Ruzzante C
                                   (March 2023)
______________________________________________________________________________________
*/

/**
 * @brief WiFi credentials information:
 * 
 * Edit with the target AP information.
*/

#define WIFI_SSID "XXXXXXXXXXXXX"
#define WIFI_PASSWORD "XXXXXXXXX"

/**
 * @brief SoftAP credentials information:
 * 
 * Edit with the desired credentials for the softAP mode.
*/

#define WIFI_AP_SSID "ESP32 JRC Tech"
#define WIFI_AP_PASSWORD ""

/**
 * @brief Custom mac definitions:
 * 
 * Edit the mac address with the desired custom mac for station or ap.
 * 
 * Comment these lines for use factory station mac and/or factory ap mac instead.
 * 
*/

 #define JRC_WIFI_STA_CUSTOM_MAC {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66}
 #define JRC_WIFI_AP_CUSTOM_MAC  {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x67}

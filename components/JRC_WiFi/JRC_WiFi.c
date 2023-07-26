/*
______________________________________________________________________________________
                                JRC_WiFi Library V1.0
                                by:  Javier Ruzzante C
                                   (March 2023)
______________________________________________________________________________________
*/

/*
    Source file: Contains the library function implementations.

    See the JRC_WiFi.h or the documentation file for reference.
*/

#include "JRC_WiFi.h"
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "secrets.h"

static const char *TAG = "JRC_WiFi";

JRC_WiFi_Status_t WiFi_Status = JRC_WIFI_NOT_STARTED;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        WiFi_Status = JRC_WIFI_STARTED;
        ESP_LOGI(TAG, "Connecting to AP...");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        WiFi_Status = JRC_WIFI_CONNECTED;
        ESP_LOGI(TAG, "Connected to AP");
        break;
    case IP_EVENT_STA_GOT_IP:
        WiFi_Status = JRC_WIFI_GOT_IP;
        ESP_LOGI(TAG, "Got IP address");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        WiFi_Status = JRC_WIFI_DISCONNECTED;
        ESP_LOGI(TAG, "Disconnected from AP");
        esp_wifi_connect();
        break;

    case WIFI_EVENT_AP_START:
        WiFi_Status = JRC_WIFI_AP_STARTED;
        ESP_LOGI(TAG, "WIFI_EVENT_AP_START");
        break;
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    break;
    case WIFI_EVENT_AP_STACONNECTED:
    {
        ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED");
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    break;
    default:
        break;
    }
}



esp_err_t JRC_WiFi_Begin()
{
    nvs_flash_init();

    if (JRC_WiFi_Get_Status() != JRC_WIFI_NOT_STARTED)
    {
        ESP_LOGW(TAG, "Stop previously initialized WiFi...");
        JRC_WiFi_Stop();
        ESP_LOGW(TAG, "WiFi stopped. Init again...");
    }


    // Stage 1. Wi-Fi/LwIP Init Phase:

     esp_netif_init();
     esp_event_loop_create_default();
     esp_netif_ptr= esp_netif_create_default_wifi_sta();

     wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
     esp_wifi_init(&wifi_init_cfg);

    #ifdef JRC_WIFI_USE_RAM_STORAGE
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    #endif

    // Stage 2. Wi-Fi Configuration Phase:
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_sta_config_t wifi_sta_cfg = {};

    strcpy((char *)wifi_sta_cfg.ssid, WIFI_SSID);
    strcpy((char *)wifi_sta_cfg.password, WIFI_PASSWORD);

    wifi_config_t wifi_cfg = {.sta = wifi_sta_cfg};

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));

    #ifdef JRC_WIFI_STA_CUSTOM_MAC
    uint8_t customMac[] = JRC_WIFI_STA_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_STA, customMac);
    #endif

    // Stage 3. Wi-Fi Start Phase:
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

esp_err_t JRC_WiFi_AP_Info(wifi_ap_record_t *apinfo)
{
    wifi_ap_record_t ap_info;

    esp_wifi_sta_get_ap_info(&ap_info);

    if (apinfo == NULL)
    {
        ESP_LOGI(TAG, "SSID: %s", ap_info.ssid);
        ESP_LOGI(TAG, "RSSI: %d", ap_info.rssi);
        ESP_LOGI(TAG, "BSSID: %02x:%02x:%02x:%02x:%02x:%02x",
                 ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
                 ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
        ESP_LOGI(TAG, "Channel: %d", ap_info.primary);
    }
    else
    {
        memcpy(apinfo, &ap_info, sizeof(wifi_ap_record_t));
    }

    return ESP_OK;
}

esp_err_t JRC_WiFi_STA_MAC_Address(char *mac)
{
    uint8_t mac_address[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_address);

    if (mac == NULL)
    {
        ESP_LOGI(TAG, "Station MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_address[0], mac_address[1], mac_address[2],
                 mac_address[3], mac_address[4], mac_address[5]);
    }
    else
    {
        sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x%c", mac_address[0], mac_address[1], mac_address[2],
                mac_address[3], mac_address[4], mac_address[5], '\0');
    }
    return ESP_OK;
}

esp_err_t JRC_WiFi_STA_IP_Address(char *ipAdd)
{
    /* Obtain the default network interface */
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    /* Obtain the IP info from the network interface */

    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret == ESP_OK)
    {
        if (ipAdd == NULL)
        {
            /* Show IP address */
            char buff[IP_ADDR_LENGTH];
            esp_ip4addr_ntoa(&ip_info.ip, buff, IP_ADDR_LENGTH);
            ESP_LOGI(TAG, "IP Address: %s", buff);
        }
        else
        {
            esp_ip4addr_ntoa(&ip_info.ip, ipAdd, IP_ADDR_LENGTH);
        }
    }

    return ret;
}

esp_err_t JRC_WiFi_STA_Subnet_Mask(char *snMask)
{
    /* Obtain the default network interface */
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    /* Obtain the IP info from the network interface */

    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret == ESP_OK)
    {
        if (snMask == NULL)
        {
            /* Show Subnet Mask */
            char buff[IP_ADDR_LENGTH];
            esp_ip4addr_ntoa(&ip_info.netmask, buff, IP_ADDR_LENGTH);
            ESP_LOGI(TAG, "Subnet Mask: %s", buff);
        }
        else
        {
            esp_ip4addr_ntoa(&ip_info.netmask, snMask, IP_ADDR_LENGTH);
        }
    }

    return ret;
}

esp_err_t JRC_WiFi_STA_Gateway(char *gateWay)
{
    /* Obtain the default network interface */
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    /* Obtain the IP info from the network interface */

    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret == ESP_OK)
    {
        if (gateWay == NULL)
        {
            /* Show Gateway */
            char buff[IP_ADDR_LENGTH];
            esp_ip4addr_ntoa(&ip_info.gw, buff, IP_ADDR_LENGTH);
            ESP_LOGI(TAG, "Gateway: %s", buff);
        }
        else
        {
            esp_ip4addr_ntoa(&ip_info.gw, gateWay, IP_ADDR_LENGTH);
        }
    }

    return ret;
}

JRC_WiFi_Status_t JRC_WiFi_Get_Status()
{
    return WiFi_Status;
}

esp_err_t JRC_WiFi_Stop()
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    esp_netif_t *netif_sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif_sta != NULL) {
        esp_netif_destroy(netif_sta);
    }

    esp_netif_t *netif_ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif_ap != NULL) {
        esp_netif_destroy(netif_ap);
    }

    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler);

    WiFi_Status = JRC_WIFI_NOT_STARTED;
    return ESP_OK;
}

esp_err_t JRC_WiFi_Begin_AP()
{
    nvs_flash_init();

    if (JRC_WiFi_Get_Status() != JRC_WIFI_NOT_STARTED)
    {
        ESP_LOGW(TAG, "Stop previously initialized WiFi...");
        JRC_WiFi_Stop();
        ESP_LOGW(TAG, "WiFi stopped. Init again...");
    }


    // Stage 1. Wi-Fi/LwIP Init Phase:

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);

    #ifdef JRC_WIFI_USE_RAM_STORAGE
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    #endif

    // Stage 2. Wi-Fi Configuration Phase:
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    strcpy((char *)wifi_ap_config.ap.ssid, WIFI_AP_SSID);
    strcpy((char *)wifi_ap_config.ap.password, WIFI_AP_PASSWORD);

    if (strlen(WIFI_AP_PASSWORD) == 0)
    {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    #ifdef JRC_WIFI_AP_CUSTOM_MAC
    uint8_t customMac[] = JRC_WIFI_AP_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_AP, customMac);
    #endif

    // Stage 3. Wi-Fi Start Phase:
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

esp_err_t JRC_WiFi_Begin_STA_AP()
{
    nvs_flash_init();

    if (JRC_WiFi_Get_Status() != JRC_WIFI_NOT_STARTED)
    {
        ESP_LOGW(TAG, "Stop previously initialized WiFi...");
        JRC_WiFi_Stop();
        ESP_LOGW(TAG, "WiFi stopped. Init again...");
    }

    // Stage 1. Wi-Fi/LwIP Init Phase:

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);

    #ifdef JRC_WIFI_USE_RAM_STORAGE
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    #endif

    // Stage 2. Wi-Fi Configuration Phase:

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_sta_config_t wifi_sta_cfg = {};

    strcpy((char *)wifi_sta_cfg.ssid, WIFI_SSID);
    strcpy((char *)wifi_sta_cfg.password, WIFI_PASSWORD);

    wifi_ap_config_t wifi_ap_cfg = {
        .ssid_len = strlen(WIFI_AP_SSID),
        .channel = 0,
        .max_connection = 4,
        .authmode = WIFI_AUTH_WPA2_PSK,
        .pmf_cfg = {
            .required = false,
        },
    };

    strcpy((char *)wifi_ap_cfg.ssid, WIFI_AP_SSID);
    strcpy((char *)wifi_ap_cfg.password, WIFI_AP_PASSWORD);

    if (strlen(WIFI_AP_PASSWORD) == 0)
    {
        wifi_ap_cfg.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config_t wifi_sta_config = {.sta = wifi_sta_cfg};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    
    #ifdef JRC_WIFI_STA_CUSTOM_MAC
    uint8_t customMacSta[] = JRC_WIFI_STA_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_STA, customMacSta);
    #endif

    wifi_config_t wifi_ap_config = {.ap = wifi_ap_cfg};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    #ifdef JRC_WIFI_AP_CUSTOM_MAC
    uint8_t customMacAP[] = JRC_WIFI_AP_CUSTOM_MAC;
    esp_wifi_set_mac(WIFI_IF_AP, customMacAP);
    #endif

    // Stage 3. Wi-Fi Start Phase:
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

static void print_auth_mode(int authmode)
{
    switch (authmode)
    {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_OWE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OWE");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    default:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher)
    {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }

    switch (group_cipher)
    {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }
}

esp_err_t JRC_WiFi_Scan(void *ap_found_info)
{
    if (WiFi_Status == JRC_WIFI_NOT_STARTED)
    {
        nvs_flash_init();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
        assert(sta_netif);

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        #ifdef JRC_WIFI_USE_RAM_STORAGE
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        #endif

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

        #ifdef JRC_WIFI_STA_CUSTOM_MAC
        uint8_t customMac[] = JRC_WIFI_STA_CUSTOM_MAC;
        esp_wifi_set_mac(WIFI_IF_STA, customMac);
        #endif

        ESP_ERROR_CHECK(esp_wifi_start());
        WiFi_Status = JRC_WIFI_STARTED;
    }

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    if (ap_found_info == NULL)
    {
        ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
        for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++)
        {
            ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
            ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
            print_auth_mode(ap_info[i].authmode);
            if (ap_info[i].authmode != WIFI_AUTH_WEP)
            {
                print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
            }
            ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
        }
    }
    else
    {
        for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++)
        {
            ((wifi_ap_record_t *)ap_found_info)[i] = ap_info[i];
        }
    }
    return ESP_OK;
}


#include <cstring>
#include <cmath>
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"  // Ensure this path is correct
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"

extern "C"
{
#include "st7789.h"
#include "fontx.h"
}

#define ESP_CHANNEL 1
uint8_t peer_mac[6]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55};//direccion mac del otro esp32
uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x56};//direccion mac de este esp32

uint8_t msg[24];
char int_string[24];
char data_string[24];

// VARIABLES DEL GIROSCOPIO
static const char *TAG = "ESP"; // Define a tag for logging
#define MAC_ADDR_SIZE 6
float value;

//funcion que procesa los datos que recibe del otro esp32
void recv_cb(const esp_now_recv_info_t * esp_now_info,const uint8_t *data, int data_len){
	sprintf(data_string,"%s", data);
	ESP_LOGI(TAG, "%s", data);
	value = strtof(data_string, NULL); // Convierte la cadena a float
	//se agrega el dato recibido a una variable global de tipo string
}
//funcion que se ejecuta al enviar un dato
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
	if(status==ESP_NOW_SEND_SUCCESS){
		ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
	}
	else{
		ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
	}
	//avisa por consola si el dato se envió correctamente
}
//funcion para inicializar el wifi
void init_wifi(void){
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_netif_init();
	esp_event_loop_create_default();
	nvs_flash_init();
	esp_wifi_init(&wifi_init_config);
	esp_wifi_set_mode(WIFI_MODE_STA);//modo station
	esp_wifi_start();
	ESP_LOGI(TAG, "wifi init complete");
}
//funcion para inicializar el protocolo esp-now
void init_esp_now(void){
	esp_now_init();
	esp_now_register_recv_cb(recv_cb);//se agregan las funciones que se ejecutaran al recibir o enviar datos
	esp_now_register_send_cb(send_cb);
	ESP_LOGI(TAG, "esp now init complete");
}
//funcion para configurar la direccion mac del receptor
void register_peer(uint8_t *peer_addr){
	esp_now_peer_info_t esp_now_peer_info={};
	memcpy(esp_now_peer_info.peer_addr, peer_mac, 6);
	esp_now_peer_info.channel=ESP_CHANNEL;
	esp_now_peer_info.ifidx=WIFI_IF_STA;
	esp_now_add_peer(&esp_now_peer_info);
	ESP_LOGI(TAG, "register peer init complete");
}
//funcion para encviar datos al otro esp32
void send_data(const uint8_t *data, size_t len) {
    // Enviar datos al dispositivo receptor
    esp_now_send(NULL, data, len);
}

static void get_mac_address()
{
    uint8_t mac[MAC_ADDR_SIZE];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI("MAC address", "MAC address: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void set_mac_address(uint8_t *mac)
{
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, mac);
    if (err == ESP_OK) {
        ESP_LOGI("MAC address", "MAC address successfully");
    } else {
        ESP_LOGE("MAC address", "Failed to set MAC address");
    }
}

void SPIFFS_Directory(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}

extern "C" void app_main(){
	//----------------------------------------------PANTALLA----------------------------------------------------------------
	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 12,
		.format_if_mount_failed =true
	};
	esp_vfs_spiffs_register(&conf);
	SPIFFS_Directory("/spiffs/");
	FontxFile fx24G[2];
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT","");
	TFT_t lcd;
	spi_master_init(&lcd, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO,CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	lcdInit(&lcd, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
	//----------------------------------------------GIROSCOPIO----------------------------------------------------------------

	int data[110]={0};

	init_wifi();
	init_esp_now();
	register_peer(peer_mac);

	set_mac_address(mac_address);
	get_mac_address();

    while (true){
    	data[109]=value*-1;

       	lcdFillScreen(&lcd, BLACK);

    	for(int i=0; i<109; i++){
    		lcdDrawLine(&lcd, 20+i*2, 140+data[i]*2, i*2+22, 140+data[i+1]*2, GREEN);
    	}
       	lcdDrawFillRect(&lcd,17, 16,19,280, RED);
       	lcdDrawFillRect(&lcd,0, 261,280,263, CYAN);

       	sprintf(int_string,"%.2f", value);
       	strcpy((char *)msg, int_string);
       	lcdDrawString(&lcd, fx24G, 25, 62, msg, GREEN);

    	lcdDrawFinish(&lcd);

    	for(int i=0;i<109;i++){
    		data[i]=data[i+1];
    	}
    	data[109]=0;

    	//delayMS(100);
    }
}

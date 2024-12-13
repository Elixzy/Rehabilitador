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
uint8_t peer_mac[6]={0x00, 0x11, 0x22, 0x33, 0x44, 0x56};//direccion mac del otro esp32
uint8_t mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};//direccion mac de este esp32

uint8_t msg[24];
char int_string[24];
// PINES DE SENTIDO DE GIRO PARA EL PUENTE H
#define ARRIBA GPIO_NUM_44
#define ABAJO GPIO_NUM_3
// CONFIGURACION I2C
#define I2C_MASTER_SCL 10
#define I2C_MASTER_SDA 11
#define I2C_MASTER_NUM I2C_NUM_0
#define QMI8658_ADDRESS 0x6B // Replace with your QMI8658 address
// ENCODER PIN Y CONTADOR
#define ENCODER_PIN GPIO_NUM_18
volatile int pulse_count = 0;
// VARIABLES DEL GIROSCOPIO
static const char *TAG = "QMI8658"; // Define a tag for logging
SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;
float pitch = 0.0, roll = 0.0;
// VARIABLE DEL ADC
int adc_raw;
// VARIABLES DE RUTINA
int rutina =0;
#define MAC_ADDR_SIZE 6

//funcion que procesa los datos que recibe del otro esp32
void recv_cb(const esp_now_recv_info_t * esp_now_info,
		const uint8_t *data, int data_len){
	sprintf(int_string,"%s", data);
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

void IRAM_ATTR encoder_isr_handler(void *arg) {
    pulse_count++;
}

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // I2C clock frequency
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void read_sensor_data(void* arg) {
    while (1) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                ESP_LOGI(TAG, "ACCEL: %f, %f, %f", acc.x, acc.y, acc.z);
            } else {
                ESP_LOGE(TAG, "Failed to read accelerometer data");
            }

            if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
                ESP_LOGI(TAG, "GYRO: %f, %f, %f", gyr.x, gyr.y, gyr.z);
            } else {
                ESP_LOGE(TAG, "Failed to read gyroscope data");
            }

            ESP_LOGI(TAG, "Timestamp: %u, Temperature: %.2f *C", (unsigned int)qmi.getTimestamp(), qmi.getTemperature_C()); // Casting to unsigned int
        } else {
            ESP_LOGW(TAG, "Data not ready yet");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup_sensor() {
    i2c_master_init();

    // Initialize QMI8658 sensor with 4 parameters (port number, address, SDA, SCL)
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, I2C_MASTER_SDA, I2C_MASTER_SCL)) {
        ESP_LOGE(TAG, "Failed to find QMI8658 - check your wiring!");
        vTaskDelete(NULL); // Handle error gracefully
    }

    // Get chip ID
    ESP_LOGI(TAG, "Device ID: %x", qmi.getChipID());

    // Configure accelerometer
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_0,
        true
    );
    // Configure gyroscope
    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    // Enable gyroscope and accelerometer
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(TAG, "Ready to read data...");
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

void rutinas(void *arg){
	//----------------------------------------------PWM----------------------------------------------------------------
	ledc_channel_config_t ledc_channel2 = {
		.gpio_num       = 2,
	    .speed_mode     = LEDC_LOW_SPEED_MODE,
	    .channel        = LEDC_CHANNEL_0,
	    .intr_type      = LEDC_INTR_DISABLE,
	    .timer_sel      = LEDC_TIMER_0,
	    .duty           = 0,
	    .hpoint         = 0
	};

	ledc_channel_config_t ledc_channel1 = {
		.gpio_num       = 16,
	    .speed_mode     = LEDC_LOW_SPEED_MODE,
	    .channel        = LEDC_CHANNEL_1,
	    .intr_type      = LEDC_INTR_DISABLE,
	    .timer_sel      = LEDC_TIMER_0,
	    .duty           = 0,
	    .hpoint         = 0
	};

	ledc_timer_config_t ledc_timer = {
	    .speed_mode       = LEDC_LOW_SPEED_MODE,
	    .duty_resolution  = LEDC_TIMER_12_BIT,
	    .timer_num        = LEDC_TIMER_0,
	    .freq_hz          = 5000,
	    .clk_cfg          = LEDC_AUTO_CLK
	};

	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_channel1);
	ledc_channel_config(&ledc_channel2);

	while(true){
		if(rutina==1){
			for(int i=0;i<5;i++){
				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 850);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

				while(roll<=10){}

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 800);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

				while(roll>=-10){}

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 850);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

				while(roll<=0){}

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			}
			rutina =0;
		}

		else if(rutina==2){
			for(int i=0;i<5;i++){
				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 830);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

				while(roll<=5){}

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 800);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

				while(roll>=-5){}

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 830);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

				while(roll<=0){}

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
			}
			rutina =0;
		}
	}
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
	setup_sensor();
	//----------------------------------------------PINES----------------------------------------------------------------
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<ENCODER_PIN) |
						(1ULL<<ARRIBA) |
						(1ULL<<ABAJO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_PIN, encoder_isr_handler, NULL);
    //----------------------------------------------ADC----------------------------------------------------------------
	adc_oneshot_unit_handle_t adc2_handle;

	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_2,};
	adc_oneshot_new_unit(&init_config1, &adc2_handle);

	adc_oneshot_chan_cfg_t config = {
		.atten = ADC_ATTEN_DB_11,
		.bitwidth = ADC_BITWIDTH_12,
		};

	adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_6, &config);

	xTaskCreate(rutinas, "rutinas", 1024*6, NULL, 1, NULL);

	int data[110]={0};

	init_wifi();
	init_esp_now();
	register_peer(peer_mac);

	set_mac_address(mac_address);
	get_mac_address();

    while (true){
    	adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &adc_raw);

    	if(!gpio_get_level(ARRIBA)){
    		rutina=1;
    	}
    	else if(!gpio_get_level(ABAJO)){
    		rutina=2;
    	}

    	if (qmi.getDataReady()){
    		qmi.getAccelerometer(acc.x, acc.y, acc.z);
    	}

    	pitch = atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180 / M_PI;
    	roll = atan2(acc.y, sqrt(acc.x * acc.x + acc.z * acc.z)) * 180 / M_PI;

    	//message="a\n";
		sprintf(int_string,"%.2f\n", roll);
		send_data((const uint8_t *)int_string, strlen(int_string));

    	data[109]=roll*-1;

       	lcdFillScreen(&lcd, BLACK);

    	for(int i=0; i<109; i++){
    		lcdDrawLine(&lcd, 20+i*2, 140+data[i]*2, i*2+22, 140+data[i+1]*2, GREEN);
    	}
       	lcdDrawFillRect(&lcd,17, 16,19,280, RED);
       	lcdDrawFillRect(&lcd,0, 261,280,263, CYAN);

       	sprintf(int_string,"%d", (adc_raw*100)/4095);
       	strcpy((char *)msg, int_string);
       	lcdDrawString(&lcd, fx24G, 25, 32, msg, PURPLE);

       	sprintf(int_string,"%.2f", roll);
       	strcpy((char *)msg, int_string);
       	lcdDrawString(&lcd, fx24G, 25, 62, msg, GREEN);

       	/*sprintf(int_string,"%d", pulse_count);
       	strcpy((char *)msg, int_string);
       	lcdDrawString(&lcd, fx24G, 25, 92, msg, PURPLE);*/

       	sprintf(int_string,"R: %d", rutina);
       	strcpy((char *)msg, int_string);
       	lcdDrawString(&lcd, fx24G, 25, 252, msg, WHITE);

    	lcdDrawFinish(&lcd);

    	for(int i=0;i<109;i++){
    		data[i]=data[i+1];
    	}
    	data[109]=0;

    	//delayMS(50);
    }
}

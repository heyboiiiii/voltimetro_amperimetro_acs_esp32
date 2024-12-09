#include <stdio.h>
#include "string.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "sdkconfig.h"

#include "LCD_i2c/lcd.h"

#define ADC_PIN1 4  // GPIO del que se va a leer corriente (bateria)
#define ADC_PIN2 5 // GPIO del que se va a leer voltaje (bateria)

//resistencias del divisor resistivo para leer la salida del sensor de corriente(ACS712)
float R1 = 1000.0;
float R2 = 2000.0;//reemplazar valores en caso de ser diferente resistencias

//resistencias del divisor resistivo para leer la tension de la bateria
float R3 = 4700.0;
float R4 = 10000.0;//reemplazar valores en caso de ser diferente resistencias

float adc_value;float CURRENT_BATTERY;float VOLTAGE_BATTERY;
float* p = &adc_value;

//tag del logi
static const char* TAG = "ADC>";


adc_oneshot_unit_handle_t adc_handle;// handle del adc
adc_oneshot_chan_cfg_t config_channel = {
  .bitwidth = ADC_BITWIDTH_DEFAULT,
  .atten = ADC_ATTEN_DB_12,
}; // creo el struct para la config de los canales


////////////////////////////////////////////////////////////////////////////////////

// Definiciones de pines

#define I2C_MASTER_SCL_IO           14      /*!< Pin SCL (Reloj) */
#define I2C_MASTER_SDA_IO           27    /*!< Pin SDA (Datos) */
#define TEST_I2C_PORT              0
//#define I2C_CLK_SRC_DEFAULT        100000 /*!< Frecuencia de I2C */
#define LCD_ADDR                    0x27    /*!< Dirección I2C del LCD (puede variar) */


// configurar el LCD
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = TEST_I2C_PORT,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;


i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = LCD_ADDR,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t dev_handle;





//////////////////////////////////////////////////////////////////////////////


float readvoltage(){

  adc_oneshot_read(adc_handle, ADC_PIN2, &p);

  float adc_voltage = adc_value * (3.3 / 4095.0); // 4095 for 12-bit ADC

        // Calculate voltage
  return (adc_voltage * (R1 + R2) / R2);
}




float readcurrent_acs712(){
      // Read the ADC value

        adc_oneshot_read(adc_handle, ADC_PIN1, &p);

        // Calculate voltage
        float adc_voltage = adc_value * (3.3 / 4095.0); // 4095 for 12-bit ADC

        // Calculate current
        float current_voltage = (adc_voltage * (R1 + R2) / R2);
        return (current_voltage - 2.5) / 0.066;// 0.066 = Sensitivy se saca del datasheet ACS20-> 100mv/A && ACS30 -> 66mv/A.
}









void app_main(void) {
  adc_oneshot_unit_init_cfg_t config_adc; //struct de config de canales
  config_adc.unit_id=ADC_UNIT_1;
  config_adc.ulp_mode= ADC_ULP_MODE_DISABLE;
  config_adc.clk_src= 0;

  adc_oneshot_new_unit(&config_adc, &adc_handle);//creo la unidad con el handle y config
  
  adc_channel_t c1; adc_channel_t c2;
  //obtengo el channel para el pin que voy a leer
    //I => ADC_PIN1
    adc_oneshot_io_to_channel(ADC_PIN1,NULL,&c1);
    adc_oneshot_config_channel(adc_handle, c1, &config_channel);//configuro el channel
    //V => ADC_PIN2
    adc_oneshot_io_to_channel(ADC_PIN2,NULL,&c2);
    adc_oneshot_config_channel(adc_handle, c2, &config_channel);//configuro el channel
  
  




  //muestro el canal asginado para cada pin.
  ESP_LOGI("ADC1","pin: %d >> channel: %d",ADC_PIN1,c1);
  ESP_LOGI("ADC2","pin: %d >> channel: %d",ADC_PIN1,c2);



  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    lcd_init(dev_handle, 0x27);
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    // Limpio LCD
    lcd_string("starting esp32");





/*

 lcd_init();
       lcd_send_cmd(0x01); // Borrar pantalla
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x80); // Posición del cursor en la primera línea
    lcd_display_text("Voltaje: ");
    lcd_send_cmd(0xC0); // Posición del cursor en la segunda línea
    lcd_display_text("Corriente: ");
*/
  

    while (1) {

        CURRENT_BATTERY = readcurrent_acs712();
        ESP_LOGI(TAG, "Current Value: %.2f%%. A", CURRENT_BATTERY);




        VOLTAGE_BATTERY = readcurrent_acs712();
        ESP_LOGI(TAG, "Current Value: %.2f%%. A", VOLTAGE_BATTERY);


        // Mostrar los valores en el LCD
        char voltage_text[16];
        snprintf(voltage_text, sizeof(voltage_text), "V: %.2f V", VOLTAGE_BATTERY);
        lcd_set_cursor(0, 0);// Mover cursor a la primera línea
        lcd_string(voltage_text);

        char current_text[16];
        snprintf(current_text, sizeof(current_text), "I: %.2f A", CURRENT_BATTERY);
        lcd_set_cursor(0, 1);// Mover cursor a la segunda línea
        lcd_string(current_text);



        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




#include <stdio.h>
#include "freertos/FreeRTOS.h" // data types
#include "freertos/task.h"     // vTaskDelay
#include "esp_log.h"

#include "bno055.h"


void app_main()
{
    printf("\n\n\n");
    printf("*******************\n");
    printf("    BNO055 test\n");
    printf("*******************\n");
    
    bno055_config_t bno_conf;
    i2c_number_t i2c_num = 0;
    
    esp_log_level_set("bno055", ESP_LOG_DEBUG);

    esp_err_t err;
    err = bno055_set_default_conf( &bno_conf); 
    printf("bno055_set_default_conf() returned 0x%02X \n", err);
    
    err = bno055_open(i2c_num, &bno_conf);
    printf("bno055_open() returned 0x%02X \n", err);
    
    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        goto end_prog;
    }
    vTaskDelay(1000 / portTICK_RATE_MS);

    bno055_chip_info_t chip_inf;
    err = bno055_get_chip_info(i2c_num, &chip_inf);
    printf("bno055_get_chip_info() returned 0x%02X \n", err);
    vTaskDelay(10 / portTICK_RATE_MS);
    
    if( err == ESP_OK) bno055_displ_chip_info(chip_inf);

    err = bno055_set_opmode(i2c_num, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(1000 / portTICK_RATE_MS);

    uint8_t system_status;
    err = bno055_get_system_status(i2c_num, &system_status);
    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        goto end_prog;
    }
    printf("System status: 0x%02X \n", system_status);

    uint8_t self_test_result;
    err = bno055_get_self_test_result(i2c_num, &self_test_result);
    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        goto end_prog;
    }
    printf("Self test result: 0x%02X \n", self_test_result);

    uint8_t system_error;
    err = bno055_get_system_error(i2c_num, &system_error);
    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        goto end_prog;
    }
    printf("System error: 0x%02X \n", system_error);
    
    uint8_t temperature;
    err = bno055_get_temperature(i2c_num, &temperature);
    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        goto end_prog;
    }
    printf("Temperature: %d \n", temperature);

end_prog:
    err = bno055_close(i2c_num);
    printf("bno055_close() returned 0x%02X \n", err);
    printf("Finished\n");
 
}

  
 

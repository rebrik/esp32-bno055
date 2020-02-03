#include <stdio.h>
#include "freertos/FreeRTOS.h" // data types
#include "freertos/task.h"     // vTaskDelay
#include "esp_log.h"

#include "bno055.h"


void app_main()
{
    printf("\n\n\n");
    printf("*******************************\n");
    printf("    BNO055 calibration test\n");
    printf("*******************************\n");
    
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

    err = bno055_set_opmode(i2c_num, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(1000 / portTICK_RATE_MS);
    
    printf("\n    Gyroscope Calibration\n");
    printf("Place the device in a single stable position for a period \n");
    printf("of few seconds to allow the gyroscope to calibrate\n");
    printf("\n    Accelerometer Calibration\n");
    printf("Place the device in 6 different stable positions for a period of \n");
    printf("few seconds to allow the accelerometer to calibrate.\n");
    printf("Make sure that there is slow movement between 2 stable positions\n");
    printf("The 6 stable positions could be in any direction, but make sure that \n");
    printf("the device is lying at least once perpendicular to the x, y and z axis.\n");  
    printf("\n    Magnetometer Calibration\n");
    printf("Make some random movements (for example: writing the number ‘8’ on air) \n");
    printf("until the CALIB_STAT register indicates fully calibrated.\n");

    printf("\nCalibration Status\n");
    printf("     i\t\tcalib\tsystem\tgyro\taccel\tmag\n");

    uint8_t calib, system, gyro, accel, mag;
    int i = 0;
    while(1){
        
        err = bno055_get_calib_status_byte(i2c_num, &calib);
        err = bno055_get_calib_status(i2c_num, &system, &gyro, &accel, &mag);
        if( err != ESP_OK ) {
            printf("Program terminated!\n");
            goto end_prog;
        }

        printf("%6d\t\t0x%X\t%1d\t%1d\t%1d\t%1d\n", i, calib, system, gyro, accel, mag);
        i++;
        vTaskDelay(500 / portTICK_RATE_MS);
    }
    
end_prog:
    err = bno055_close(i2c_num);
    printf("bno055_close() returned 0x%02X \n", err);
    printf("Finished\n");
 
}

  
 

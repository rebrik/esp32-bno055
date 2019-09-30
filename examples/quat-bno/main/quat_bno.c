#include <stdio.h>
#include "freertos/FreeRTOS.h" // data types
#include "freertos/task.h"     // vTaskDelay

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
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
    
//    esp_log_level_set("*", ESP_LOG_INFO);

    esp_err_t err;
    err = bno055_set_default_conf( &bno_conf); 
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

    int n_quats = 100;
    printf("\n\nGetting %d quaternions...\n\n", n_quats);

    //TODO wait for calibration

    printf("endheader\n");  // marker for python animation
    printf("t\tw\tx\ty\tz\n");

    int64_t time_mks;
    bno055_quaternion_t quat;
    for(int i=0; i<n_quats; i++){
       vTaskDelay(50 / portTICK_RATE_MS);
       err = bno055_get_quaternion(i2c_num, & quat);
        if( err != ESP_OK ) {
            printf("bno055_get_quaternion() returned error: %02x \n", err);
            if(err == 107) continue; // i2c_timeout
        }
        time_mks = esp_timer_get_time();
        printf("%lld\t",time_mks);
        printf("%.6f\t%.6f\t%.6f\t%.6f\n", quat.w, quat.x, quat.y, quat.z );
    }
    printf("Finished\n");
    
end_prog:
    err = bno055_close(i2c_num);
    printf("bno055_close() returned 0x%02X \n", err);
 
}

  
 

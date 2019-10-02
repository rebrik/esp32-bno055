#include <stdio.h>
#include "freertos/FreeRTOS.h" // data types
#include "freertos/task.h"     // vTaskDelay

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include "bno055.h"

// BNO polling period. At 100Hz scheduler rate should be a multiple of 10 ms
#define BNO_POLLING_MS 10

void quat_task( void *pvParameters ){

	// The xLastWakeTime variable needs to be initialized with the current tick count.
	// Note that this is the only time the variable is explicitly written to.
	// After this xLastWakeTime is managed automatically by the vTaskDelayUntil()API function.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t err;
    int64_t time_mks, time_mks_after;
    int time_bno;
    bno055_quaternion_t quat;
    i2c_number_t i2c_num = * (i2c_number_t *)pvParameters; // convert from void *

    while( 1 ) {
		time_mks = esp_timer_get_time();
		err = bno055_get_quaternion(i2c_num, & quat);
		time_mks_after = esp_timer_get_time();
		if( err != ESP_OK ) {
			printf("bno055_get_quaternion() returned error: %02x \n", err);
			exit(2);
		}
		time_bno = time_mks_after - time_mks;
		printf("%16lld\t%10d\t",time_mks, time_bno);
		printf("%.6f\t%.6f\t%.6f\t%.6f\n", quat.w, quat.x, quat.y, quat.z );

    	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( BNO_POLLING_MS ));
    }
}

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
        err = bno055_close(i2c_num);
        printf("bno055_close() returned 0x%02X \n", err);
        exit(1);
    }

    vTaskDelay(1000 / portTICK_RATE_MS);

    err = bno055_set_opmode(i2c_num, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(1000 / portTICK_RATE_MS);

    //TODO wait for calibration

    TaskHandle_t xHandle = NULL;
    // create task on the APP CPU (CPU_1)
    err = xTaskCreatePinnedToCore(quat_task,   // task function
    		                      "quat_task",    // task name for debugging
								   2048,          // stack size bytes
								   &i2c_num,      // pointer to params to pass
								   10,            // task priority. configMAX_PRIORITIES == 25, so 24 is the highest. 0 is idle
								   &xHandle,      // returned task handle
								   1);            // CPU to use 1 means APP_CPU

    printf("endheader\n");  // marker for python animation
    printf("t\tdt\tw\tx\ty\tz\n");

}

  
 

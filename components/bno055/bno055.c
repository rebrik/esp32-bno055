
#include <stdio.h>
#include <string.h> // memset()
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "bno055.h"

typedef enum
{

    BNO055_CHIP_ID_ADDR                                     = 0x00,
    BNO055_ACCEL_ID_ADDR                                    = 0x01,
    BNO055_MAG_ID_ADDR                                      = 0x02,
    BNO055_GYRO_ID_ADDR                                     = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
    BNO055_BL_REV_ID_ADDR                                   = 0X06,
    BNO055_PAGE_ID_ADDR                                     = 0X07,

	// Data
    BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,
    BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,
    BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,
    BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
    BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
    BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
    BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
    BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
    BNO055_EULER_P_MSB_ADDR                                 = 0X1F,
    BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,
    BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

	BNO055_TEMP_ADDR                                        = 0X34,

    // Status
    BNO055_CALIB_STAT_ADDR                                  = 0X35,
    BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
    BNO055_INTR_STAT_ADDR                                   = 0X37,

    BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
    BNO055_SYS_STAT_ADDR                                    = 0X39,
    BNO055_SYS_ERR_ADDR                                     = 0X3A,

    // Units selection
    BNO055_UNIT_SEL_ADDR                                    = 0X3B,
    BNO055_DATA_SELECT_ADDR                                 = 0X3C,

    // Mode selection
    BNO055_OPR_MODE_ADDR                                    = 0X3D,
    BNO055_PWR_MODE_ADDR                                    = 0X3E,

    BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
    BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

    // Axis remapping
    BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

    // Soft iron calibration matrix 3x3
    BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

    // Accelerometer Offsets
    ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

    // Magnetometer Offsets
    MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
    MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

    // Gyroscope Offsets
    GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
    GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

    // Radius
    ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
    ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
    MAG_RADIUS_LSB_ADDR                                     = 0X69,
    MAG_RADIUS_MSB_ADDR                                     = 0X6A
} bno055_reg_t;



#define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer 
#define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer 

#define WRITE_BIT I2C_MASTER_WRITE  // I2C master write 
#define READ_BIT I2C_MASTER_READ    // I2C master read 
#define ACK_CHECK_EN 0x1            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           // I2C master will not check ack from slave 
#define ACK_VAL 0x0                 // I2C ack value 
#define NACK_VAL 0x1                // I2C nack value 

   
static const char *TAG = "bno055";
   
    
typedef struct
{
    bno055_addr_t  i2c_address; // BNO055_ADDRESS_A or BNO055_ADDRESS_B
    bool  bno_is_open;
} bno055_device_t;


static bno055_device_t x_bno_dev[I2C_NUM_MAX];

static uint8_t x_buffer[200];  // we so far are using only 20 bytes max

// Internal functions

// _______________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack |
// --------|-----------------------------------|-------------------------|
// ________________________________________________________________________
// | start | write chip_addr + rd_bit, chk_ack | read 1 byte, nack | stop |
// --------|-----------------------------------|-------------------|------|
esp_err_t bno055_read_register(i2c_number_t i2c_num, bno055_reg_t reg, uint8_t *p_reg_val){
    
    if( !x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGE(TAG, "bno055_read_register(): device is not open");
        return BNO_ERR_NOT_OPEN; //TODO: make error list
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // making the command - begin 
    i2c_master_start(cmd);  // start condition
    // device address with write bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN); 
    // send the register address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);   
    i2c_master_start(cmd);  // start condition again
    // device address with read bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | READ_BIT, ACK_CHECK_EN);   
    // read byte, issue NACK
    i2c_master_read_byte(cmd, p_reg_val, NACK_VAL); 
    i2c_master_stop(cmd);  // stop condition
    // making the command - end 
    
    // Now execute the command
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    switch (err) {
        case ESP_OK: 
            break;
        case  ESP_ERR_TIMEOUT:
            ESP_LOGE(TAG, "bno055_read_register(): i2c timeout");
            break;
        default: 
            ESP_LOGE(TAG, "bno055_read_register(): failed");
    }
    
    return err;   
}
  
// ______________________________________________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack | write 1 byte, chk_ack | stop |
// --------|-----------------------------------|-------------------------|-----------------------|------|

 esp_err_t bno055_write_register(i2c_number_t i2c_num, bno055_reg_t reg, uint8_t reg_val){
    
    if( !x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGE(TAG, "bno055_write_register(): device is not open");
        return BNO_ERR_NOT_OPEN;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // making the command - begin 
    i2c_master_start(cmd);  // start condition
    // device address with write bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN); 
    // send the register address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);   
    // write byte, issue NACK
    i2c_master_write_byte(cmd, reg_val, ACK_CHECK_EN); 
    i2c_master_stop(cmd);  // stop condition
    // making the command - end 
    
    // Now execute the command
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    switch (err) {
        case ESP_OK: 
            break;
        case  ESP_ERR_TIMEOUT:
            ESP_LOGE(TAG, "bno055_write_register(): i2c timeout");
            break;
        default: 
            ESP_LOGE(TAG, "bno055_write_register(): failed");
    }
    
    return err;   
}
 
// _______________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack |
// --------|-----------------------------------|-------------------------|
// ______________________________________________________________________________________________
// | start | write chip_addr + rd_bit, chk_ack | read n-1 bytes, ack | read 1 byte, nack | stop |
// --------|-----------------------------------|---------------------|-------------------|------|
 
esp_err_t bno055_read_data(i2c_number_t i2c_num, bno055_reg_t start_reg, uint8_t *buffer, uint8_t n_bytes){

    if( !x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGE(TAG, "bno055_read_data(): device is not open");
        return BNO_ERR_NOT_OPEN;
    }

    if( n_bytes < 2 || n_bytes > 0x7F ) {
        ESP_LOGE(TAG, "bno055_read_data(): invalid number of bytes: %d", n_bytes);
        return BNO_ERR_NOT_IN_RANGE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // making the command - begin 
    i2c_master_start(cmd);  // start condition
    // device address with write bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN); 
    // send the register address
    i2c_master_write_byte(cmd, start_reg, ACK_CHECK_EN);   
    i2c_master_start(cmd);  // start condition again
    // device address with read bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | READ_BIT, ACK_CHECK_EN);
    // read n_bytes-1, issue ACK
    i2c_master_read(cmd, buffer, n_bytes - 1, ACK_VAL);
    // read the last byte, issue NACK
    i2c_master_read_byte(cmd, buffer+n_bytes-1, NACK_VAL); 
    i2c_master_stop(cmd);  // stop condition
    // making the command - end 

    // Now execute the command
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    switch (err) {
        case ESP_OK: 
        	break;
        case ESP_ERR_TIMEOUT:
            ESP_LOGE(TAG, "bno055_read_data(): i2c timeout");
            break;
        default: 
            ESP_LOGE(TAG, "bno055_read_data(): failed");
    }
    
    return err;   
}


// Public functions

esp_err_t bno055_set_default_conf(bno055_config_t * p_bno_conf){

    p_bno_conf->i2c_address = BNO055_ADDRESS_A;          // BNO055_ADDRESS_A or BNO055_ADDRESS_B 
    p_bno_conf->sda_io_num = 18;        // GPIO number for I2C sda signal 25
    p_bno_conf->sda_pullup_en = GPIO_PULLUP_DISABLE;  // Internal GPIO pull mode for I2C sda signal
    p_bno_conf->scl_io_num = 19;        // GPIO number for I2C scl signal 26 
    p_bno_conf->scl_pullup_en = GPIO_PULLUP_DISABLE;  // Internal GPIO pull mode for I2C scl signal
    p_bno_conf->clk_speed = 400000;     // I2C clock frequency for master mode, (no higher than 1MHz for now) 
    p_bno_conf->timeout = 10000*80;     // 10ms in 80 MHz ticks, should be < 0xFFFFF
    p_bno_conf->use_ext_oscillator = true; // Use external oscillator

    return ESP_OK;   
}



esp_err_t bno055_open(i2c_number_t i2c_num, bno055_config_t * p_bno_conf )
{
    if(i2c_num >= I2C_NUMBER_MAX) return ESP_ERR_INVALID_ARG;
    
    // Check if already in use
    if(x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGW(TAG, "bno055_open(): device is already open");
        return BNO_ERR_ALREADY_OPEN;
    }
    
    x_bno_dev[i2c_num].bno_is_open = 0;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = p_bno_conf->sda_io_num;        
    conf.sda_pullup_en = p_bno_conf->sda_pullup_en;  
    conf.scl_io_num = p_bno_conf->scl_io_num;        
    conf.scl_pullup_en = p_bno_conf->scl_pullup_en;  
    conf.master.clk_speed = p_bno_conf->clk_speed;
    
    esp_err_t err;
    
    err = i2c_param_config(i2c_num, &conf);
    ESP_LOGD(TAG, "i2c_param_config() returned 0x%02X", err);
    if( err != ESP_OK ) return err;
        
    err = i2c_driver_install(i2c_num, I2C_MODE_MASTER,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_LOGD(TAG, "i2c_driver_install() returned 0x%02X", err);
    if( err != ESP_OK ) return err;
    
    err = i2c_set_timeout(i2c_num, p_bno_conf->timeout);
    ESP_LOGD(TAG, "i2c_set_timeout() returned 0x%02X", err);
    if( err != ESP_OK ) return err;
    
    x_bno_dev[i2c_num].i2c_address = p_bno_conf->i2c_address;
    
    // Read BNO055 Chip ID to make sure we have a connection
    x_bno_dev[i2c_num].bno_is_open = 1; // bno055_read_register() checks this flag
    uint8_t reg_val;
    err = bno055_read_register(i2c_num, BNO055_CHIP_ID_ADDR, & reg_val);
    
    if( err == ESP_OK ) {
        
        ESP_LOGD(TAG, "BNO055 ID returned 0x%02X", reg_val);
        if( reg_val == BNO055_ID ) {
            ESP_LOGI(TAG, "BNO055 detected \n");
        } 
        else {
            ESP_LOGE(TAG, "bno055_open() error: BNO055 NOT detected");
            goto errExit;
        }
    }
    
    
     // Switch to config mode
    err = bno055_write_register(i2c_num, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if(err != ESP_OK) goto errExit;
    vTaskDelay(30 / portTICK_RATE_MS);
    ESP_LOGD(TAG, "Set config oper. mode - Ok");

    // Reset
    err=bno055_write_register(i2c_num, BNO055_SYS_TRIGGER_ADDR, 0x20 );
    if(err != ESP_OK) goto errExit;
    vTaskDelay(700 / portTICK_RATE_MS);
    ESP_LOGD(TAG, "BNO055 reset - Ok");

    // Set ext oscillator
    if(p_bno_conf->use_ext_oscillator) {
    	err=bno055_write_register(i2c_num, BNO055_SYS_TRIGGER_ADDR, 0x80);
        if(err != ESP_OK) goto errExit;
        vTaskDelay(20 / portTICK_RATE_MS);
    }
    ESP_LOGD(TAG, "Set external oscillator - Ok");
    
    // TODO: turn off sleep mode
    
    // Set power mode to normal 
    err=bno055_write_register(i2c_num, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    if(err != ESP_OK) goto errExit;
    vTaskDelay(20 / portTICK_RATE_MS);
    ESP_LOGD(TAG, "Set normal power mode - Ok");

    return ESP_OK;   
    
errExit:
    bno055_close (i2c_num);
    return err;
}



esp_err_t bno055_close (i2c_number_t i2c_num )
{
    x_bno_dev[i2c_num].bno_is_open = 0;
    return i2c_driver_delete(i2c_num);
  
}

esp_err_t bno055_get_chip_info(i2c_number_t i2c_num, bno055_chip_info_t* chip_inf){
    
    memset(chip_inf, 0, sizeof(bno055_chip_info_t));
    
    esp_err_t err = bno055_read_data(i2c_num, BNO055_CHIP_ID_ADDR, x_buffer, 7);
    if( err != ESP_OK ) return err;
    
    chip_inf->chip_id = x_buffer[0];
    chip_inf->accel_id = x_buffer[1];
    chip_inf->mag_id = x_buffer[2];
    chip_inf->gyro_id = x_buffer[3];
    chip_inf->sw_rev = x_buffer[4] + ((uint16_t)x_buffer[5]<<8);
    chip_inf->bl_rev = x_buffer[6];
    
    return ESP_OK;
}   

void bno055_displ_chip_info(bno055_chip_info_t chip_inf){

    printf("BNO055 Chip ID (0xA0): 0x%02X \n", chip_inf.chip_id );
    printf("Accelerometer Chip ID (0xFB): 0x%02X \n", chip_inf.accel_id );
    printf("Magnetometer Chip ID (0x32): 0x%02X \n", chip_inf.mag_id );
    printf("Gyroscope Chip ID (0x0F): 0x%02X \n", chip_inf.gyro_id );
    printf("Software Revision: %d \n", chip_inf.sw_rev );
    printf("Bootloader Revision: %d \n", chip_inf.bl_rev );

}   

esp_err_t bno055_set_opmode(i2c_number_t i2c_num, bno055_opmode_t mode ){
    
    esp_err_t err=bno055_write_register(i2c_num, BNO055_OPR_MODE_ADDR, mode);
    vTaskDelay(30 / portTICK_RATE_MS);
    return err;
}


esp_err_t bno055_get_opmode(i2c_number_t i2c_num, bno055_opmode_t * mode ){
    
    uint8_t ui_mode;
    esp_err_t err = bno055_read_register(i2c_num, BNO055_OPR_MODE_ADDR, &ui_mode);
    ui_mode = ui_mode & 0x0F; // upper 4 bits are reserved, lower 4 represent the mode
    * mode = ui_mode;
    return err;
}

// Note: should be in config mode to work!
esp_err_t bno055_set_ext_crystal_use(i2c_number_t i2c_num, bool use_ext ){
    
    bno055_opmode_t mode;
    esp_err_t err = bno055_get_opmode(i2c_num, & mode );
    if( err != ESP_OK ) return err;
    
    if( mode  != OPERATION_MODE_CONFIG ) {
        ESP_LOGE(TAG, "bno055_set_ext_crystal_use(): device should be in the config mode. Current mode: %d", mode);
        return BNO_ERR_WRONG_OPMODE;
    }

    uint8_t reg_val;
    if(use_ext) reg_val = 0x80;
    else reg_val = 0;

    // Set ext crystal on/off 
    err=bno055_write_register(i2c_num, BNO055_SYS_TRIGGER_ADDR, reg_val);
    vTaskDelay(10 / portTICK_RATE_MS);
    return err;
}

esp_err_t bno055_get_system_status(i2c_number_t i2c_num,uint8_t *system_status){

	esp_err_t err;
	err = bno055_read_register(i2c_num, BNO055_SYS_STAT_ADDR, system_status);

	vTaskDelay(30 / portTICK_RATE_MS);
	return err;
}

esp_err_t bno055_get_self_test_result(i2c_number_t i2c_num, uint8_t *self_test_result){

	esp_err_t err;
	err = bno055_read_register(i2c_num, BNO055_SELFTEST_RESULT_ADDR, self_test_result);

	vTaskDelay(30 / portTICK_RATE_MS);
	return err;
}

esp_err_t bno055_get_system_error(i2c_number_t i2c_num, uint8_t *system_error){

	esp_err_t err;
	err = bno055_read_register(i2c_num, BNO055_SYS_ERR_ADDR, system_error);

	vTaskDelay(30 / portTICK_RATE_MS);
	return err;
}

esp_err_t bno055_get_temperature(i2c_number_t i2c_num, uint8_t* p_temperature){

  esp_err_t err = bno055_read_register(i2c_num, BNO055_TEMP_ADDR, p_temperature);
  return err;
}

esp_err_t bno055_get_calib_status_byte(i2c_number_t i2c_num, uint8_t* calib) {
	esp_err_t err;
	err = bno055_read_register(i2c_num, BNO055_CALIB_STAT_ADDR, calib);
    return err;
}

esp_err_t bno055_get_calib_status(i2c_number_t i2c_num, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    
    uint8_t calib_status;
	esp_err_t err;
	err = bno055_read_register(i2c_num, BNO055_CALIB_STAT_ADDR, &calib_status);
    if( err != ESP_OK ) return err;
   
    *sys = (calib_status >> 6) & 0x03;
    *gyro = (calib_status >> 4) & 0x03;
    *accel = (calib_status >> 2) & 0x03;
    *mag = calib_status & 0x03;
    
    return ESP_OK;
}

esp_err_t _bno055_buf_to_quaternion(uint8_t *buffer, bno055_quaternion_t* quat) {

    int16_t x, y, z, w;
    // combine MSB and LSB into 16-bit int
    w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    const double conv_coeff = (1.0 / (1<<14));
    quat->w = conv_coeff * w;
    quat->x = conv_coeff * x;
    quat->y = conv_coeff * y;
    quat->z = conv_coeff * z;

    return ESP_OK;
}

esp_err_t bno055_get_quaternion(i2c_number_t i2c_num, bno055_quaternion_t* quat) {

    esp_err_t err = bno055_read_data(i2c_num, BNO055_QUATERNION_DATA_W_LSB_ADDR, x_buffer, 8);
    if( err != ESP_OK ) return err;

    return _bno055_buf_to_quaternion(x_buffer, quat);
}

esp_err_t _bno055_buf_to_lin_accel(uint8_t *buffer, bno055_vec3_t* lin_accel) {

    int16_t x, y, z;
    // combine MSB and LSB into 16-bit int
    x = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    y = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    z = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);

    const double conv_coeff = 1e-2;  // we assume m/s^2 units
    lin_accel->x = conv_coeff * x;
    lin_accel->y = conv_coeff * y;
    lin_accel->z = conv_coeff * z;

    return ESP_OK;
}

esp_err_t bno055_get_lin_accel(i2c_number_t i2c_num, bno055_vec3_t* lin_accel) {

    esp_err_t err = bno055_read_data(i2c_num, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, x_buffer, 6);
    if( err != ESP_OK ) return err;
    
    return _bno055_buf_to_lin_accel(x_buffer, lin_accel);
}

esp_err_t _bno055_buf_to_gravity(uint8_t *buffer, bno055_vec3_t* gravity) {

    int16_t x, y, z;
    // combine MSB and LSB into 16-bit int
    x = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    y = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    z = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);

    const double conv_coeff = 1e-2;  // we assume m/s^2 units
    gravity->x = conv_coeff * x;
    gravity->y = conv_coeff * y;
    gravity->z = conv_coeff * z;

    return ESP_OK;
}

esp_err_t bno055_get_gravity(i2c_number_t i2c_num, bno055_vec3_t* gravity){
    esp_err_t err = bno055_read_data(i2c_num, BNO055_GRAVITY_DATA_X_LSB_ADDR, x_buffer, 6);
    if( err != ESP_OK ) return err;
    
    return _bno055_buf_to_gravity(x_buffer, gravity);
}

esp_err_t bno055_get_fusion_data(i2c_number_t i2c_num, bno055_quaternion_t* quat, bno055_vec3_t* lin_accel, bno055_vec3_t* gravity){

    esp_err_t err = bno055_read_data(i2c_num, BNO055_QUATERNION_DATA_W_LSB_ADDR, x_buffer, 20);
    if( err != ESP_OK ) return err;
    
    _bno055_buf_to_quaternion(x_buffer, quat);
    _bno055_buf_to_lin_accel(x_buffer+8, lin_accel);
    _bno055_buf_to_gravity(x_buffer+14, gravity);
    
    return ESP_OK;
}



#ifndef _BNO055_H_
#define _BNO055_H_

#include "driver/gpio.h"  // gpio_num_t, gpio_pullup_t

typedef enum{
    I2C_NUMBER_0 = 0,  // I2C port 0
    I2C_NUMBER_1 ,     // I2C port 1
    I2C_NUMBER_MAX
} i2c_number_t;

typedef enum
{
    BNO055_ADDRESS_A = 0x28,
    BNO055_ADDRESS_B = 0x29
} bno055_addr_t;

// Chip ID
#define BNO055_ID                (0xA0)

// ESP-BNO errors
#define BNO_ERR_NOT_OPEN         (0xB05501)
#define BNO_ERR_ALREADY_OPEN     (0xB05502)
#define BNO_ERR_NOT_IN_RANGE     (0xB05503)
#define BNO_ERR_WRONG_OPMODE     (0xB05504)

typedef enum
{
    POWER_MODE_NORMAL            = 0X00,
    POWER_MODE_LOWPOWER          = 0X01,
    POWER_MODE_SUSPEND           = 0X02
} bno055_powermode_t;

typedef enum
{
    // Operation modes
    OPERATION_MODE_CONFIG        = 0X00,
    OPERATION_MODE_ACCONLY       = 0X01,
    OPERATION_MODE_MAGONLY       = 0X02,
    OPERATION_MODE_GYRONLY       = 0X03,
    OPERATION_MODE_ACCMAG        = 0X04,
    OPERATION_MODE_ACCGYRO       = 0X05,
    OPERATION_MODE_MAGGYRO       = 0X06,
    OPERATION_MODE_AMG           = 0X07,
    OPERATION_MODE_IMUPLUS       = 0X08,
    OPERATION_MODE_COMPASS       = 0X09,
    OPERATION_MODE_M4G           = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF  = 0X0B,
    OPERATION_MODE_NDOF          = 0X0C
} bno055_opmode_t;

typedef enum
{
    REMAP_CONFIG_P0              = 0x21,
    REMAP_CONFIG_P1              = 0x24, // default
    REMAP_CONFIG_P2              = 0x24,
    REMAP_CONFIG_P3              = 0x21,
    REMAP_CONFIG_P4              = 0x24,
    REMAP_CONFIG_P5              = 0x21,
    REMAP_CONFIG_P6              = 0x21,
    REMAP_CONFIG_P7              = 0x24
} bno055_axis_remap_config_t;

typedef enum
{
    REMAP_SIGN_P0                = 0x04,
    REMAP_SIGN_P1                = 0x00, // default
    REMAP_SIGN_P2                = 0x06,
    REMAP_SIGN_P3                = 0x02,
    REMAP_SIGN_P4                = 0x03,
    REMAP_SIGN_P5                = 0x01,
    REMAP_SIGN_P6                = 0x07,
    REMAP_SIGN_P7                = 0x05
} bno055_axis_remap_sign_t;

typedef struct
{
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;

    int16_t accel_radius;
    int16_t mag_radius;
} bno055_offsets_t;

typedef struct {
    uint8_t i2c_address;          // BNO055_ADDRESS_A or BNO055_ADDRESS_B 
    gpio_num_t sda_io_num;        // GPIO number for I2C sda signal 
    gpio_pullup_t sda_pullup_en;  // Internal GPIO pull mode for I2C sda signal
    gpio_num_t scl_io_num;        // GPIO number for I2C scl signal 
    gpio_pullup_t scl_pullup_en;  // Internal GPIO pull mode for I2C scl signal
    uint32_t clk_speed;           // I2C clock frequency: 100000 or 400000
    int timeout;                  // in 80 MHz ticks, should be <= 0xFFFFF
    bool use_ext_oscillator;      // Use external oscillator
} bno055_config_t;

typedef struct {
    uint8_t  chip_id;
    uint8_t  accel_id;
    uint8_t  mag_id;
    uint8_t  gyro_id;
    uint16_t sw_rev;
    uint8_t  bl_rev;
} bno055_chip_info_t;

typedef struct {
    double  w;
    double  x;
    double  y;
    double  z;
} bno055_quaternion_t;

typedef struct {
    double  x;
    double  y;
    double  z;
} bno055_vec3_t;


esp_err_t bno055_set_default_conf(bno055_config_t * p_bno_conf);

esp_err_t bno055_open(i2c_number_t i2c_num, bno055_config_t * p_bno_conf );
esp_err_t bno055_close (i2c_number_t i2c_num );
esp_err_t bno055_get_chip_info(i2c_number_t i2c_num, bno055_chip_info_t* chip_inf);
     void bno055_displ_chip_info(bno055_chip_info_t chip_inf);
esp_err_t bno055_set_opmode(i2c_number_t i2c_num, bno055_opmode_t mode );
esp_err_t bno055_get_opmode(i2c_number_t i2c_num, bno055_opmode_t * mode );
esp_err_t bno055_set_ext_crystal_use(i2c_number_t i2c_num, bool use_ext );
esp_err_t bno055_get_temperature(i2c_number_t i2c_num, uint8_t* temperature);

//   System Status
//   ---------------------------------
//   0 = Idle
//   1 = System Error
//   2 = Initializing Peripherals
//   3 = System Initialization
//   4 = Executing Self-Test
//   5 = Sensor fusion algorithm running
//   6 = System running without fusion algorithms
esp_err_t bno055_get_system_status(i2c_number_t i2c_num,uint8_t *system_status);

//	Self Test Results
//	--------------------------------
//	1 = test passed, 0 = test failed
//
//	Bit 0 = Accelerometer self test
//	Bit 1 = Magnetometer self test
//	Bit 2 = Gyroscope self test
//	Bit 3 = MCU self test
//
//	0x0F = all tests OK
esp_err_t bno055_get_self_test_result(i2c_number_t i2c_num, uint8_t *self_test_result);

//	System Error
//	---------------------------------
//	0 = No error
//	1 = Peripheral initialization error
//	2 = System initialization error
//	3 = Self test result failed
//	4 = Register map value out of range
//	5 = Register map address out of range
//	6 = Register map write error
//	7 = BNO low power mode not available for selected operat ion mode
//	8 = Accelerometer power mode not available
//	9 = Fusion algorithm configuration error
//	A = Sensor configuration error
esp_err_t bno055_get_system_error(i2c_number_t i2c_num, uint8_t *system_error);

esp_err_t bno055_displ_sys_status(i2c_number_t i2c_num);


// each field can have values from 0 to 3
// 3 indicates fully calibrated; 0 indicates not calibrated
// uint8_t* system - Current system calibration status, depends on status of all sensors
esp_err_t bno055_get_calib_status(i2c_number_t i2c_num, uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
esp_err_t bno055_get_calib_status_byte(i2c_number_t i2c_num, uint8_t* calib);

esp_err_t bno055_get_quaternion(i2c_number_t i2c_num, bno055_quaternion_t* quat);
esp_err_t bno055_get_lin_accel(i2c_number_t i2c_num, bno055_vec3_t* lin_accel);
esp_err_t bno055_get_gravity(i2c_number_t i2c_num, bno055_vec3_t* gravity);

esp_err_t bno055_get_fusion_data(i2c_number_t i2c_num, bno055_quaternion_t* quat, bno055_vec3_t* lin_accel, bno055_vec3_t* gravity);


#endif // _BNO055_H_


#ifndef BMP280_LIB_H
#define BMP280_LIB_H

#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @file bmp280_lib.h
 * @brief Header file for BMP280 temperature and pressure sensor driver
 *
 * This file contains the declarations for interfacing with the BMP280 sensor,
 * including register definitions, configuration structures, and function prototypes.
 */

/**
 * @defgroup BMP280_Registers BMP280 Register Addresses
 * @{
 */
#define BMP280_SENSOR_ADDR 0x76    /**< I2C address of BMP280 sensor */
#define BMP280_REG_TEMP_XLSB 0xFC  /**< Temperature data XLSB register */
#define BMP280_REG_TEMP_LSB 0xFB   /**< Temperature data LSB register */
#define BMP280_REG_TEMP_MSB 0xFA   /**< Temperature data MSB register */
#define BMP280_REG_PRESS_XLSB 0xF9 /**< Pressure data XLSB register */
#define BMP280_REG_PRESS_LSB 0xF8  /**< Pressure data LSB register */
#define BMP280_REG_PRESS_MSB 0xF7  /**< Pressure data MSB register */
#define BMP280_REG_CONFIG 0xF5     /**< Configuration register */
#define BMP280_REG_CTRL_MEAS 0xF4  /**< Control measurement register */
#define BMP280_REG_STATUS 0xF3     /**< Status register */
#define BMP280_REG_RESET 0xE0      /**< Reset register */
#define BMP280_REG_ID 0xD0         /**< Chip ID register */
/** @} */

/**
 * @defgroup BMP280_Modes Operating Modes
 * @{
 */
#define MODE_NORMAL 0 /**< Continuous measurement mode */
#define MODE_FORCED 1 /**< Single measurement mode */
#define MODE_SLEEP 2  /**< No measurements */
/** @} */

/**
 * @defgroup BMP280_Constants Physical Constants
 * @brief Constants used for pressure normalization calculations
 * @{
 */
#define TEMPERATURE_STANDARD 288.15 /**< Standard temperature in Kelvin */
#define GRAVITY 9.80665             /**< Acceleration due to gravity in m/s² */
#define PRESSURE_SEA_LEVEL 101325   /**< Standard pressure at sea level in Pa */
#define MOLAR_MASS_AIR 0.0289644    /**< Molar mass of air in kg/mol */
#define GAS_CONSTANT 8.31446        /**< Universal gas constant in J/(mol·K) */
/** @} */

/**
 * @brief Pressure oversampling settings
 * Higher oversampling results in better resolution but higher power consumption
 */
typedef enum
{
    OSRS_P_SKIPPED = 0b000, /**< Pressure measurement skipped */
    OSRS_P_X1 = 0b001,      /**< Oversampling ×1 */
    OSRS_P_X2 = 0b010,      /**< Oversampling ×2 */
    OSRS_P_X4 = 0b011,      /**< Oversampling ×4 */
    OSRS_P_X8 = 0b100,      /**< Oversampling ×8 */
    OSRS_P_X16 = 0b101      /**< Oversampling ×16 */
} bmp280_osrs_p_t;

/**
 * @brief Temperature oversampling settings
 * Higher oversampling results in better resolution but higher power consumption
 */
typedef enum
{
    OSRS_T_SKIPPED = 0b000, /**< Temperature measurement skipped */
    OSRS_T_X1 = 0b001,      /**< Oversampling ×1 */
    OSRS_T_X2 = 0b010,      /**< Oversampling ×2 */
    OSRS_T_X4 = 0b011,      /**< Oversampling ×4 */
    OSRS_T_X8 = 0b100,      /**< Oversampling ×8 */
    OSRS_T_X16 = 0b101      /**< Oversampling ×16 */
} bmp280_osrs_t_t;

/**
 * @brief IIR filter coefficient settings
 * Higher filter coefficient reduces noise but increases response time
 */
typedef enum
{
    FILTER_OFF = 0b000,    /**< Filter disabled */
    FILTER_COEF_2 = 0b001, /**< Filter coefficient 2 */
    FILTER_COEF_4 = 0b010, /**< Filter coefficient 4 */
    FILTER_COEF_8 = 0b011, /**< Filter coefficient 8 */
    FILTER_COEF_16 = 0b100 /**< Filter coefficient 16 */
} bmp280_filter_t;

/**
 * @brief Standby time settings for normal mode
 * Longer standby time results in lower power consumption
 */
typedef enum
{
    T_SB_0_5 = 0b000,  /**< 0.5 ms standby time */
    T_SB_62_5 = 0b001, /**< 62.5 ms standby time */
    T_SB_125 = 0b010,  /**< 125 ms standby time */
    T_SB_250 = 0b011,  /**< 250 ms standby time */
    T_SB_500 = 0b100,  /**< 500 ms standby time */
    T_SB_1000 = 0b101, /**< 1000 ms standby time */
    T_SB_2000 = 0b110, /**< 2000 ms standby time */
    T_SB_4000 = 0b111  /**< 4000 ms standby time */
} bmp280_t_sb_t;

/**
 * @brief Configuration structure for BMP280 initialization
 */
typedef struct
{
    i2c_port_t i2c_port; /**< I2C port number */
    uint8_t i2c_addr;    /**< I2C device address */
    uint8_t mode;        /**< Operating mode */
} bmp280_config_t;

/**
 * @brief Initializes the BMP280 sensor with given configuration
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, or appropriate error code
 *
 * Performs complete sensor initialization including:
 * - Verifying sensor ID
 * - Performing soft reset
 * - Reading calibration data
 * - Setting initial configuration
 */
esp_err_t bmp280_init(bmp280_config_t *config);

/**
 * @brief Reads temperature and pressure measurements from the sensor
 * @param temperature Pointer to store temperature in degrees Celsius
 * @param pressure Pointer to store pressure in Pascals
 * @return ESP_OK on success, or appropriate error code
 */
esp_err_t bmp280_read_measurements(float *temperature, float *pressure);

/**
 * @brief Sets the operating mode of the sensor
 * @param mode Desired operating mode (MODE_NORMAL, MODE_FORCED, or MODE_SLEEP)
 * @return ESP_OK on success, or appropriate error code
 */
esp_err_t bmp280_set_mode(uint8_t mode);

/**
 * @brief Configures sensor measurement parameters
 * @param osrs_p Pressure oversampling setting
 * @param osrs_t Temperature oversampling setting
 * @param filter IIR filter coefficient
 * @param t_sb Standby time for normal mode
 * @return ESP_OK on success, or appropriate error code
 */
esp_err_t bmp280_configure_regs(bmp280_osrs_p_t osrs_p, bmp280_osrs_t_t osrs_t,
                                bmp280_filter_t filter, bmp280_t_sb_t t_sb);

/**
 * @brief Verifies communication with the sensor
 * @return ESP_OK if connected and ID matches, ESP_ERR_INVALID_RESPONSE otherwise
 */
esp_err_t bmp280_check_connection(void);

/**
 * @brief Calculates sea-level equivalent pressure
 * @param pressure Measured pressure in Pascals
 * @param altitude Current altitude in meters
 * @return Calculated sea-level pressure in Pascals
 */
double normalize_pressure(double pressure, double altitude);

#endif // BMP280_LIB_H
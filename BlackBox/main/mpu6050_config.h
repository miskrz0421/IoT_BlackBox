// mpu6050_config.h

#ifndef MPU6050_CONFIG_H
#define MPU6050_CONFIG_H

#include "mpu6050.h"
#include "esp_err.h"

// Registers
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_PWR_MGMT_1 0x6B

// Typ ładunku - określa czułość akcelerometru
typedef enum
{
    CARGO_VERY_DELICATE = 0, // bardzo delikatny (szkło, elektronika) -> 2G
    CARGO_STANDARD = 1,      // standardowy (ubrania, książki) -> 4G
    CARGO_DURABLE = 2,       // wytrzymały (narzędzia) -> 8G
    CARGO_VERY_DURABLE = 3   // bardzo wytrzymały (sprzęt budowlany) -> 16G
} cargo_type_t;

// Wymagania pozycji - określa czułość żyroskopu
typedef enum
{
    POSITION_STRICT = 0,      // musi być w pozycji pionowej -> 250DPS
    POSITION_PREFERRED = 1,   // preferowana określona pozycja -> 500DPS
    POSITION_NO_THROWING = 2, // nie może być rzucana -> 1000DPS
    POSITION_ANY = 3          // dowolna pozycja -> 2000DPS
} position_requirements_t;

// Środek transportu - wpływa na filtrację
typedef enum
{
    TRANSPORT_AIR = 0,         // samolot -> 44Hz
    TRANSPORT_TRUCK_TRAIN = 1, // ciężarówka/pociąg -> 94Hz
    TRANSPORT_COURIER = 2      // kurier/dostawa miejska -> 184Hz
} transport_type_t;

// Czas transportu - wpływa na filtrację
typedef enum
{
    DURATION_SHORT = 0,  // do 24 godzin -> mniejsza filtracja
    DURATION_MEDIUM = 1, // 1-3 dni -> standardowa filtracja
    DURATION_LONG = 2    // 3-5 dni -> większa filtracja
} transport_duration_t;

// Struktura konfiguracji otrzymywana przez MQTT
typedef struct
{
    cargo_type_t cargo_type;          // Typ ładunku
    position_requirements_t position; // Wymagania pozycji
    transport_type_t transport;       // Środek transportu
    transport_duration_t duration;    // Czas transportu
} blackbox_config_t;

/**
 * @brief Parsuje konfigurację z JSONa otrzymanego przez MQTT
 *
 * @param json_string String zawierający konfigurację w formacie JSON
 * @param config Wskaźnik do struktury, gdzie zostanie zapisana konfiguracja
 * @return esp_err_t ESP_OK w przypadku sukcesu
 */
esp_err_t blackbox_parse_config_json(const char *json_string, blackbox_config_t *config);

/**
 * @brief Konfiguruje MPU6050 na podstawie otrzymanych ustawień
 *
 * @param sensor Handle do czujnika MPU6050
 * @param config Wskaźnik do struktury z konfiguracją
 * @return esp_err_t ESP_OK w przypadku sukcesu
 */
esp_err_t blackbox_configure_mpu6050(mpu6050_handle_t sensor, const blackbox_config_t *config);

#endif // MPU6050_CONFIG_H
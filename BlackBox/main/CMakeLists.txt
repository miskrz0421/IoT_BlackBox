set(PARTITION_TABLE_CSV "partitions.csv")

idf_component_register(SRCS "mqtt.c" "wifi.c" "ble.c" "ky026.c" "main.c" "bmp280_lib.c" "mpu6050.c" "mpu6050_config.c" "sensor_storage.c" 
                    INCLUDE_DIRS ".")


                    
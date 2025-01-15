#ifndef MQTT_H
#define MQTT_H
#include <stdbool.h>
void mqtt_init(void);
bool mqtt_verify_pin(char *pin_value);
void send_control_signal(bool start);
void mqtt_publish_data(char *json, char *name);
#endif

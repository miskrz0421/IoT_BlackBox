#ifndef BLE_H
#define BLE_H
#include <stdbool.h>

void ble_init(void);
bool ble_enabled();
void ble_toggle(bool);
void ble_init_once();

#endif

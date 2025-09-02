#ifndef BLE_H
#define BLE_H

#include <stdbool.h>

void ble_init(void);
void ble_send_orientation(float roll, float pitch, float yaw);
bool ble_is_connected(void);

#endif // BLE_H

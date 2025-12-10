#ifndef PLAYERWATER_H_
#define PLAYERWATER_H_


#include <stdint.h>
#include "lcd.h"


#define PLAYER_WATER_BITS_PER_PIXEL 1
#define PLAYER_WATER_LENGTH 1024
#define PLAYER_WATER_W 32
#define PLAYER_WATER_H 32



extern const color_t player_water_left0[PLAYER_WATER_LENGTH];
extern const color_t player_water_right0[PLAYER_WATER_LENGTH];

// extern const color_t player_water_left1[PLAYER_WATER_LENGTH];
// extern const color_t player_water_right1[PLAYER_WATER_LENGTH];

#endif //PLAYERWATER
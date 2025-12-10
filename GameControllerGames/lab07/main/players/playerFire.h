#ifndef PLAYERFIRE_H_
#define PLAYERFIRE_H_


#include <stdint.h>
#include "lcd.h"

#define PLAYER_FIRE_BITS_PER_PIXEL 1
#define PLAYER_FIRE_LENGTH 1024
#define PLAYER_FIRE_W 32
#define PLAYER_FIRE_H 32

extern const color_t player_fire_left0[PLAYER_FIRE_LENGTH];
extern const color_t player_fire_right0[PLAYER_FIRE_LENGTH];

// extern const color_t player_fire_left1[PLAYER_FIRE_LENGTH];
// extern const color_t player_fire_right1[PLAYER_FIRE_LENGTH];

#endif //PLAYERFIRE
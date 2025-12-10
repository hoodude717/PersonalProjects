#include <stdint.h>
#include "playerWater.h"
#include "config.h"



#define O CONFIG_COLOR_BACKGROUND //Background COlor
#define R RED
#define B BLUE
#define G GREEN
#define P PURPLE
#define Y YELLOW
#define L BLACK
#define C CYAN



const color_t player_water_left0[] = {
    O, O, O, O, O, O, O, O, O, O, L, L, L, L, L, L, L, L, L, L, L, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, L, B, B, B, B, B, B, B, B, B, B, L, L, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, B, B, B, B, B, B, B, B, B, B, B, B, B, B, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, B, C, C, C, C, C, C, C, C, B, B, B, B, B, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, B, B, C, C, C, C, C, C, C, C, C, C, B, B, B, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, B, L, C, L, L, C, C, C, L, L, C, C, L, B, B, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, B, L, C, L, L, C, C, C, L, L, C, C, L, B, B, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, L, C, C, C, C, C, C, C, C, C, C, L, B, B, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, L, B, B, B, L, L, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, C, C, C, L, L, L, L, C, C, C, L, B, B, B, B, B, L, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, L, L, B, B, B, B, B, L, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, L, L, L, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, L, C, C, C, C, C, C, C, C, C, L, C, C, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, C, C, L, C, C, C, C, C, C, C, C, C, L, C, C, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, C, C, L, C, C, C, C, C, C, C, C, C, L, L, C, C, C, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, C, L, L, C, C, C, C, C, C, C, C, C, L, L, L, C, C, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, L, L, L, C, C, C, C, C, C, C, C, C, L, O, L, C, C, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, L, L, C, C, C, C, C, C, C, C, C, C, C, L, L, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, O, L, C, C, C, C, C, C, C, C, C, C, C, L, O, L, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, L, L, C, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, L, O, O, L, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, C, L, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, L, L, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, L, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, L, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, L, C, C, C, L, O, O, O, O, O, O, O,



};



const color_t player_water_right0[] = {
    O, O, O, O, O, O, O, O, O, O, O, L, L, L, L, L, L, L, L, L, L, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, L, L, B, B, B, B, B, B, B, B, B, L, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, B, B, B, B, B, B, B, B, B, B, B, B, B, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, B, B, B, C, C, C, C, C, C, C, C, B, B, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, B, B, B, C, C, C, C, C, C, C, C, C, B, B, L, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, B, B, L, C, L, L, C, C, C, L, L, C, B, B, L, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, B, B, L, C, L, L, C, C, C, L, L, C, L, B, L, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, B, B, L, C, C, C, C, C, C, C, C, C, L, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, L, L, B, B, B, L, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, L, B, B, B, B, B, L, C, C, L, L, L, L, C, C, C, L, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, L, B, B, B, B, B, L, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, L, L, L, L, L, L, L, L, L, L, L, L, L, L, L, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, L, L, L, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, L, C, C, C, C, C, C, C, C, C, L, C, C, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, C, C, L, C, C, C, C, C, C, C, C, C, L, C, C, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, C, C, L, C, C, C, C, C, C, C, C, C, L, L, C, C, C, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, C, L, L, C, C, C, C, C, C, C, C, C, L, L, L, C, C, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, C, L, L, L, C, C, C, C, C, C, C, C, C, L, O, L, C, C, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, L, L, C, C, C, C, C, C, C, C, C, C, C, L, L, C, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, O, L, C, C, C, C, C, C, C, C, C, C, C, L, O, L, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, C, C, C, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, C, L, L, C, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, C, C, C, C, C, L, O, O, L, C, C, C, C, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, C, L, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, L, L, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, L, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, L, C, C, C, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, C, C, C, C, L, O, O, O, O, O, O, L, C, C, C, L, O, O, O, O, O, O, O,


};

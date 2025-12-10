#include <stdint.h>
#include "playerFire.h"
#include "config.h"


#define O CONFIG_COLOR_BACKGROUND //Background COlor
#define R RED
#define B BLUE
#define G GREEN
#define P PURPLE
#define Y YELLOW
#define L BLACK


const color_t player_fire_left0[] = {
    O, O, O, O, O, O, O, O, O, L, L, O, O, L, L, O, O, O, O, L, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, L, O, O, L, R, L, O, O, O, L, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, L, O, L, R, R, L, L, L, L, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, L, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, L, L, R, R, R, L, L, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, L, L, R, R, R, L, L, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, R, R, R, L, L, L, L, R, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, L, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, L, L, L, L, L, L, L, L, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, L, L, L, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, L, R, R, R, R, R, R, R, R, R, L, R, R, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, R, R, L, R, R, R, R, R, R, R, R, R, L, R, R, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, R, R, L, R, R, R, R, R, R, R, R, R, L, L, R, R, R, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, R, L, L, R, R, R, R, R, R, R, R, R, L, L, L, R, R, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, L, L, L, R, R, R, R, R, R, R, R, R, L, O, L, R, R, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, L, L, R, R, R, R, R, R, R, R, R, R, R, L, L, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, O, L, R, R, R, R, R, R, R, R, R, R, R, L, O, L, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, L, L, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, L, O, O, L, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, R, L, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, L, L, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, L, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, L, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, L, R, R, R, L, O, O, O, O, O, O, O,



};

const color_t player_fire_right0[] = {
    O, O, O, O, O, O, O, O, O, L, R, L, O, L, L, O, O, O, O, L, L, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, L, O, L, R, L, O, O, O, L, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, L, O, L, R, R, L, L, L, L, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, L, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, L, L, R, R, R, L, L, R, R, L, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, L, L, R, R, R, L, L, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, R, R, R, L, L, L, L, R, R, R, R, L, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, L, L, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, O, L, L, L, L, L, L, L, L, O, O, O, O, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, L, L, L, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, L, R, R, R, R, R, R, R, R, R, L, R, R, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, R, R, L, R, R, R, R, R, R, R, R, R, L, R, R, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, R, R, L, R, R, R, R, R, R, R, R, R, L, L, R, R, R, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, R, L, L, R, R, R, R, R, R, R, R, R, L, L, L, R, R, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, R, L, L, L, R, R, R, R, R, R, R, R, R, L, O, L, R, R, L, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, L, L, R, R, R, R, R, R, R, R, R, R, R, L, L, R, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, L, L, O, L, R, R, R, R, R, R, R, R, R, R, R, L, O, L, L, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, R, R, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, R, L, L, R, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, O, L, R, R, R, R, R, L, O, O, L, R, R, R, R, L, O, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, R, L, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, L, L, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, L, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, L, R, R, R, L, O, O, O, O, O, O, O,
    O, O, O, O, O, O, O, O, L, R, R, R, R, L, O, O, O, O, O, O, L, R, R, R, L, O, O, O, O, O, O, O,

};
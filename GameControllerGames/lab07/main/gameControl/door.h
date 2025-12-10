#ifndef DOOR_H_
#define DOOR_H_

#include "lcd.h"
#include "config.h"
#include "player.h"

#define DOOR_W 16
#define DOOR_H 64
#define DOOR_X 160
#define DOOR_Y_MIN (-64)
#define DOOR_Y_MAX 0 
#define DOOR_SPEED 10


typedef struct {
    coord_t x;
    coord_t y;
    uint8_t state;
    bool open;

}door_t;

//set up the door
void door_init();

//door tick
void door_tick(player_t *player1, player_t *player2);

//Draw door
void door_draw();

//get door state
uint8_t door_get_state();

//return if door is open or closed.
bool door_open();



#endif //DOOR_H
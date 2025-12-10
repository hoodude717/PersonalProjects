#ifndef ELEVATOR_H_
#define ELEVATOR_H_
#include <stdint.h>
#include "config.h"
#include "lcd.h"

#define ELEVATOR_W 48
#define ELEVATOR_HEIGHT 16
#define ELEVATOR_X 0
#define ELEVATOR_MAX_Y 104
#define ELEVATOR_MIN_Y 68
#define ELEVATOR_SPEED 2




typedef struct {
    coord_t x;
    coord_t y;
    uint8_t state; //shows the state_e value


} elevator_t;



//elevator init command
void elevator_init();

// function to check if the buttons are pressed and then move the elevator up when off and down when flipped on
void elevator_tick();

//draw the elevator at current position
void elevator_draw();

//return the postion or level of the elevator
coord_t elevator_get_y();


#endif //ELEVATOR_H
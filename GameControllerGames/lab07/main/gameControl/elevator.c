#include "elevator.h"
#include <stdio.h>
#include "switches.h"

elevator_t elevator;

enum state_e {IDLE_ST, MOVING_DOWN_ST, MOVING_UP_ST};

//elevator init command
void elevator_init() {
    elevator.state = IDLE_ST;
    elevator.x = ELEVATOR_X;
    elevator.y = ELEVATOR_MIN_Y;

}

// function to check if the buttons are pressed and then move the elevator up when off and down when flipped on
void elevator_tick() {
    uint8_t currentState = elevator.state;

    switch (currentState)
    {
    case IDLE_ST:
        if(switches_get_lever() && (elevator.y < ELEVATOR_MAX_Y)) {
            elevator.state = MOVING_DOWN_ST;
        } else if (!switches_get_lever() && (elevator.y > ELEVATOR_MIN_Y)) {
            elevator.state = MOVING_UP_ST;
        }
        break;
    case MOVING_DOWN_ST:
        elevator.y += ELEVATOR_SPEED;
        if(elevator.y >= ELEVATOR_MAX_Y) {
            elevator.y = ELEVATOR_MAX_Y;
            elevator.state = IDLE_ST;
        } else if(!switches_get_lever()) {
            elevator.state = MOVING_UP_ST;
        }

        break;
    case MOVING_UP_ST:
        elevator.y -= ELEVATOR_SPEED;
        if(elevator.y <= ELEVATOR_MIN_Y) {
            elevator.y = ELEVATOR_MIN_Y;
            elevator.state = IDLE_ST;
        } else if(switches_get_lever()) {
            elevator.state = MOVING_DOWN_ST;
        }

        break;
    default:
        break;
    }

}

//draw the elevator at current position
void elevator_draw() {

    lcd_fillRect(elevator.x, elevator.y, ELEVATOR_W, ELEVATOR_HEIGHT, CONFIG_COLOR_ELEVATOR );
}

//return the elevator y postions
coord_t elevator_get_y() {
    return elevator.y;
}
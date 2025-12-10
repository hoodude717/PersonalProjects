#include "door.h"


door_t door;

enum state_e {IDLE_ST, MOVING_DOWN_ST, MOVING_UP_ST};


//set up the door
void door_init() {
    door.x = DOOR_X;
    door.y = DOOR_Y_MAX;
    door.state = IDLE_ST;
    door.open = false;
}

//door tick
void door_tick(player_t *player1, player_t *player2) {
    uint8_t currentState = door.state;
    bool pressed;
    if(player1->onBtn1 || player1->onBtn2 || player2->onBtn1 || player2->onBtn2) {
        pressed = true;
        door.open = true;
    } else {
        pressed = false;
        door.open = false;
    }

    switch (currentState)
    {
    case IDLE_ST:
        if(pressed && door.y >= DOOR_Y_MIN)  {
            door.state = MOVING_UP_ST;
        } else if (!pressed) {
            door.state = MOVING_DOWN_ST;
        }
        break;
    case MOVING_DOWN_ST:
        door.y += DOOR_SPEED;
        if(pressed) {
            door.state = MOVING_UP_ST;
        } else if (door.y >= DOOR_Y_MAX) {
            door.y = DOOR_Y_MAX;
            door.state = IDLE_ST;
        }
        break;
    case MOVING_UP_ST:
        door.y -= DOOR_SPEED;
        if(!pressed) {
            door.state = MOVING_DOWN_ST;
        } 
        if (door.y <= DOOR_Y_MIN) {
            door.y = DOOR_Y_MIN;
            door.state = IDLE_ST;
        }
        break;
    default:
        break;
    }

}

//Draw door
void door_draw() {
    lcd_fillRect(door.x, door.y, DOOR_W, DOOR_H, CONFIG_COLOR_DOOR);
}

//get door state
uint8_t door_get_state() {
    return door.state;
}

bool door_open() {
    return door.open;
}
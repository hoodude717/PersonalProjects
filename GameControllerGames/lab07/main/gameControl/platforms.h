#ifndef PLATFORMS_H
#define PLATFORMS_H
#include <stdint.h>
#include "player.h"

//retruns a number 0 - 2 which platform the player is on. Used for checking to see if the player needs to fall or stay grounded. 
void platforms_get_platform(player_t *player);


#endif //PLATFORMS_H


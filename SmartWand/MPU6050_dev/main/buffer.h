#ifndef BUFFER_H
#define BUFFER_H

#include "stdint.h"
#include <stdbool.h>

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define ORIGIN_X = DISPLAY_WIDTH / 2
#define ORIGIN_Y = DISPLAY_HEIGHT / 2
#define BUFFER_LENGTH (DISPLAY_HEIGHT * DISPLAY_WIDTH)

#define BITMAP_WIDTH_BYTES ((DISPLAY_WIDTH + 7) / 8)  // Ceiling division



void buffer_set_origin(float yaw, float pitch);
void buffer_on_button_release();
void buffer_init();
void buffer_draw_pos(float yaw, float pitch);
void buffer_convert_to_bitmap();
void buffer_clear_buffer();

#endif //BUFFER_C
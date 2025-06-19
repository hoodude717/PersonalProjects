#include "buffer.h"
#include "stdio.h"

uint8_t bitmap_buffer[DISPLAY_HEIGHT][BITMAP_WIDTH_BYTES] = {0};
uint8_t framebuffer[DISPLAY_HEIGHT][DISPLAY_WIDTH] = {0};

// Sensitivity factor determines how fast movement translates to pixels
float yaw_sensitivity = 8.0f;   // adjust as needed
float pitch_sensitivity = 4.0f; // adjust as needed

// Global variables
float origin_yaw = 0.0f;
float origin_pitch = 0.0f;
bool origin_set = false;

float smoothed_x = -1;
float smoothed_y = -1;

void buffer_set_origin(float yaw, float pitch) {
    origin_yaw = yaw;
    origin_pitch = pitch;
    origin_set = true;

    // Reset smoothing so it snaps to the new starting point
    smoothed_x = -1;
    smoothed_y = -1;
}

void buffer_draw_pos(float yaw, float pitch) {
    if (!origin_set) return;  // Don’t draw until origin is set

    // Relative angles
    // printf("origin: %02f\t origin: %02f\t", origin_yaw, origin_pitch);
    float rel_yaw = yaw - origin_yaw;
    float rel_pitch = pitch - origin_pitch;
    rel_yaw = -rel_yaw;
    rel_pitch = -rel_pitch;
    printf("Relative %02f\t Relative %02f\t", rel_yaw, rel_pitch);
    rel_yaw *= yaw_sensitivity;   // Scale by sensitivit
    rel_pitch *= pitch_sensitivity; // Scale by sensitivity

    // Map relative yaw/pitch to DISPLAY
    int raw_x = (int)((rel_yaw + 90.0f) / 360.0f * DISPLAY_HEIGHT);
    int raw_y = (int)((rel_pitch + 90.0f) / 180.0f * DISPLAY_HEIGHT);
    printf("Raw X: %d\t Raw Y: %d\t", raw_x, raw_y);

    // Clamp to display bounds
    raw_x = raw_x < 0 ? 0 : (raw_x >= DISPLAY_WIDTH ? DISPLAY_WIDTH - 1 : raw_x);
    raw_y = raw_y < 0 ? 0 : (raw_y >= DISPLAY_HEIGHT ? DISPLAY_HEIGHT - 1 : raw_y);

    // First time? Initialize smoothed values
    if (smoothed_x < 0 || smoothed_y < 0) {
        smoothed_x = raw_x;
        smoothed_y = raw_y;
    }

    // Apply smoothing
    float alpha = 0.2f;
    smoothed_x = alpha * raw_x + (1.0f - alpha) * smoothed_x;
    smoothed_y = alpha * raw_y + (1.0f - alpha) * smoothed_y;

    // Draw final pixel
    int draw_x = (int)smoothed_x;
    int draw_y = (int)smoothed_y;
    printf("X: %d\tY: %d\n", draw_x, draw_y);

    if (draw_x >= 0 && draw_x < DISPLAY_WIDTH && draw_y >= 0 && draw_y < DISPLAY_HEIGHT) {
        framebuffer[draw_y][draw_x] = 1;
    }
}


void buffer_init() {

}



void buffer_on_button_release() {
    origin_set = false;
}

void buffer_convert_to_bitmap() {
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int x = 0; x < DISPLAY_WIDTH; x++) {
            if (framebuffer[y][x]) {
                bitmap_buffer[y][x / 8] |= (1 << (7 - (x % 8)));  // MSB to LSB
            }
        }
    }
    //printy buffer as hex
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        printf("Row %d: ", y);
        for (int b = 0; b < BITMAP_WIDTH_BYTES; b++) {
            printf("%02x ", bitmap_buffer[y][b]);
        }
        printf("\n");
    }

    //print shape of buffer bitmap
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int b = 0; b < BITMAP_WIDTH_BYTES; b++) {
            for (int bit = 7; bit >= 0; bit--) {
                if ((bitmap_buffer[y][b] >> bit) & 0x01) {
                    printf("█"); // filled pixel
                } else {
                    printf(" "); // empty pixel
                }
            }
        }
        printf("\n"); // new line for each row
    }
}


void buffer_clear_buffer() {
    // Clear framebuffer
    for (int x = 0; x < DISPLAY_WIDTH; x++) {
        for (int y = 0; y < DISPLAY_HEIGHT; y++){
            framebuffer[y][x] = 0;
        }
    }
    // Clear bitmap
    for (int y = 0; y < DISPLAY_HEIGHT; y++) {
        for (int b = 0; b < BITMAP_WIDTH_BYTES; b++) {
            bitmap_buffer[y][b]=0;
        }
    }
}
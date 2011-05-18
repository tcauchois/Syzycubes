#ifndef TLC5947DAP_H
#define TLC5947DAP_H

// LED output functions

//8-bit red, green, blue values
void send_rgb(uint16_t red, uint16_t green, uint16_t blue);

//hue: 0-764, sat: 0-128, bri: 0-255
#define HUE_LIMIT 765
#define BRI_LIMIT 256
//we use a constant saturation
#define SATURATION 128
void send_hsb(uint16_t hue, uint16_t sat, uint16_t bri);

#endif

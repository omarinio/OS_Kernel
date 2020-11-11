#ifndef __LCD_H
#define __LCD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define WHITE 0x7FFF
#define BLACK 0x000
#define RED 0x001F
#define GREEN 0x03E0
#define BLUE 0x7C00

extern void drawRectangle(uint16_t fb[ 600 ][ 800 ], int startX, int startY, int l, int h, int colour);

extern void writeChar(uint16_t fb[ 600 ][ 800 ], unsigned char letter, int x, int y, uint16_t colour, int size);

extern void writeString(uint16_t fb[ 600 ][ 800 ], char* word, int x, int y, uint16_t colour, int size);

#endif

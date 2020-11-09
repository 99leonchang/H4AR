/*
 * oled.h
 *
 *  Created on: Nov 8, 2020
 *      Author: Leon
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "ssd1306.h"

static uint8_t mail816[16] = {
	0x1F,0xF8,0x10,0x08,0x18,0x18,0x14,0x28,0x13,0xC8,0x10,0x08,0x10,0x08,0x1F,0xF8
  };

static uint8_t bat816[16] = {
 0x0F,0xFE,0x30,0x02,0x26,0xDA,0x26,0xDA,0x26,0xDA,0x26,0xDA,0x30,0x02,0x0F,0xFE
};

// 'logo' 64x32px
static uint8_t logo[256] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x70, 0x1f, 0x00, 0xb0, 0x0a, 0x00, 0xff, 0xd0, 0xf0, 0x0f, 0x00, 0xf0, 0x0f, 0x01, 0xff, 0xf8,
  0xf0, 0x0f, 0x01, 0xf0, 0x0f, 0x81, 0xff, 0xfc, 0xf0, 0x0f, 0x03, 0xf0, 0x1f, 0x81, 0xff, 0xfc,
  0xf0, 0x0f, 0x01, 0xf0, 0x1f, 0x81, 0xe0, 0x3e, 0xf0, 0x0f, 0x07, 0xf0, 0x3f, 0xc1, 0xe0, 0x3e,
  0xf0, 0x0f, 0x0f, 0xf0, 0x1f, 0x81, 0xe0, 0x1e, 0xf0, 0x0f, 0x0f, 0xf0, 0x7f, 0xe1, 0xe0, 0x1e,
  0xf0, 0x0f, 0x1f, 0xf0, 0x7f, 0xc1, 0xe0, 0x7e, 0xff, 0xff, 0x3e, 0xf0, 0x79, 0xe1, 0xff, 0xfe,
  0xff, 0xff, 0x7e, 0xf0, 0xf9, 0xf1, 0xff, 0xfc, 0xff, 0xff, 0x7c, 0xf0, 0xff, 0xf1, 0xff, 0xf8,
  0xf0, 0x0f, 0xff, 0xf1, 0xff, 0xf1, 0xe1, 0xf0, 0xf0, 0x0f, 0xff, 0xf0, 0xff, 0xf9, 0xe1, 0xf0,
  0xf0, 0x0f, 0xff, 0xf1, 0xef, 0xf1, 0xe0, 0x78, 0xf0, 0x0f, 0xff, 0xf3, 0xe0, 0x7d, 0xe0, 0xf8,
  0xf0, 0x0f, 0x00, 0xf3, 0xc0, 0x7d, 0xe0, 0x7c, 0xf0, 0x0f, 0x00, 0xf3, 0xc0, 0x7d, 0xe0, 0x7c,
  0xf0, 0x0f, 0x00, 0xf7, 0x80, 0x3f, 0xe0, 0x3e, 0xf0, 0x0f, 0x00, 0xff, 0x80, 0x1f, 0xe0, 0x3e,
  0x70, 0x0f, 0x00, 0xff, 0x80, 0x1f, 0xe0, 0x3e, 0xe0, 0x07, 0x00, 0x33, 0x80, 0x16, 0xc0, 0x04,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// 'arrow_270deg', 32x32px
static uint8_t arrow_270deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0xfe, 0x00,
  0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x3f, 0xe0,
  0x00, 0x00, 0x1f, 0xf0, 0x1f, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xfc,
  0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xfc, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x1f, 0xf0,
  0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0xff, 0x00,
  0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_90deg', 32x32px
static uint8_t arrow_90deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00,
  0x00, 0xff, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x07, 0xfc, 0x00, 0x00,
  0x0f, 0xf8, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xfc,
  0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xfc, 0x1f, 0xff, 0xff, 0xf8, 0x0f, 0xf8, 0x00, 0x00,
  0x07, 0xfc, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00,
  0x00, 0x7f, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_180deg', 32x32px
static uint8_t arrow_180deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x03, 0xc7, 0xe3, 0xc0, 0x07, 0xe7, 0xe7, 0xe0, 0x07, 0xf7, 0xef, 0xe0, 0x07, 0xff, 0xff, 0xe0,
  0x07, 0xff, 0xff, 0xe0, 0x03, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0x80, 0x00, 0xff, 0xff, 0x00,
  0x00, 0x7f, 0xfe, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x0f, 0xf0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_0deg', 32x32px
static uint8_t arrow_0deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x0f, 0xf0, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x7f, 0xfe, 0x00,
  0x00, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0x80, 0x03, 0xff, 0xff, 0xc0, 0x07, 0xff, 0xff, 0xe0,
  0x07, 0xff, 0xff, 0xe0, 0x07, 0xf7, 0xef, 0xe0, 0x07, 0xe7, 0xe7, 0xe0, 0x03, 0xc7, 0xe3, 0xc0,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00,
  0x00, 0x07, 0xe0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_315deg', 32x32px
static uint8_t arrow_315deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfe, 0x00,
  0x00, 0x1f, 0xff, 0x00, 0x00, 0x1f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x80,
  0x00, 0x1f, 0xff, 0x80, 0x00, 0x0f, 0xff, 0x80, 0x00, 0x07, 0xff, 0x80, 0x00, 0x0f, 0xff, 0x80,
  0x00, 0x1f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x7f, 0xff, 0x80, 0x00, 0xff, 0x9f, 0x80,
  0x01, 0xff, 0x1f, 0x00, 0x01, 0xfe, 0x0e, 0x00, 0x01, 0xfc, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00,
  0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_225deg', 32x32px
static uint8_t arrow_225deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00,
  0x00, 0xf8, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x00, 0x01, 0xfe, 0x0e, 0x00, 0x01, 0xff, 0x1f, 0x00,
  0x00, 0xff, 0xbf, 0x80, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x1f, 0xff, 0x80,
  0x00, 0x0f, 0xff, 0x80, 0x00, 0x07, 0xff, 0x80, 0x00, 0x0f, 0xff, 0x80, 0x00, 0x1f, 0xff, 0x80,
  0x00, 0x3f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x1f, 0xff, 0x00,
  0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_135deg', 32x32px
static uint8_t arrow_135deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00,
  0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x70, 0x7f, 0x80, 0x00, 0xf8, 0xff, 0x80,
  0x01, 0xfd, 0xff, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xf8, 0x00,
  0x01, 0xff, 0xf0, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x01, 0xff, 0xf8, 0x00,
  0x01, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0xff, 0xf8, 0x00,
  0x00, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 'arrow_45deg', 32x32px
static uint8_t arrow_45deg[128] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf0, 0x00,
  0x00, 0xff, 0xf8, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xfc, 0x00,
  0x01, 0xff, 0xf8, 0x00, 0x01, 0xff, 0xf0, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x01, 0xff, 0xf0, 0x00,
  0x01, 0xff, 0xf8, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x01, 0xfd, 0xff, 0x00,
  0x00, 0xf8, 0xff, 0x80, 0x00, 0x70, 0x7f, 0x80, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x1f, 0x00,
  0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void oled_Init(I2C_HandleTypeDef *hi2c) {
	ssd1306_Init(hi2c);
	ssd1306_Bitmap(0, 0, &logo, 64, 32);
	ssd1306_UpdateScreen();
	// Remove in future
	HAL_Delay(1000);
}

/**
 * @brief Sets the contrast of the display.
 * @param[dir_num] ID of direction of arrow to display
 */
void oled_Display(uint8_t dir_num) {
	ssd1306_Fill(Black);
	ssd1306_Bitmap(48, 0, &bat816, 16, 8);
	switch (dir_num) {
		case 0:
			ssd1306_Bitmap(16, 0, &arrow_0deg, 32, 32);
			break;
		case 1:
			ssd1306_Bitmap(16, 0, &arrow_45deg, 32, 32);
			break;
		case 2:
			ssd1306_Bitmap(16, 0, &arrow_90deg, 32, 32);
			break;
		case 3:
			ssd1306_Bitmap(16, 0, &arrow_135deg, 32, 32);
			break;
		case 4:
			ssd1306_Bitmap(16, 0, &arrow_180deg, 32, 32);
			break;
		case 5:
			ssd1306_Bitmap(16, 0, &arrow_225deg, 32, 32);
			break;
		case 6:
			ssd1306_Bitmap(16, 0, &arrow_270deg, 32, 32);
			break;
		case 7:
			ssd1306_Bitmap(16, 0, &arrow_315deg, 32, 32);
			break;
		default:
			break;
	}
	ssd1306_UpdateScreen();
}

void oled_SetOn(const uint8_t on) {
	ssd1306_SetDisplayOn(on);
}

uint8_t oled_GetOn() {
	return ssd1306_GetDisplayOn();
}


#endif /* INC_OLED_H_ */
/*
 * ssd1306.c
 *
 *  Created on: Nov 6, 2020
 *      Author: Leon
 */

#include "ssd1306.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>  // For memcpy

// Screenbuffer
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

// Screen object
static SSD1306_t SSD1306;

static I2C_HandleTypeDef *SSD1306_I2C_PORT;

void ssd1306_Reset(void) {
    /* for I2C - do nothing */
}

// Send a byte to the command register
void ssd1306_WriteCommand(uint8_t byte) {
    HAL_I2C_Mem_Write(SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

// Send data
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_I2C_Mem_Write(SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

/* Fills the Screenbuffer with values from a given buffer of a fixed length */
SSD1306_Error_t ssd1306_FillBuffer(uint8_t* buf, uint32_t len) {
    SSD1306_Error_t ret = SSD1306_ERR;
    if (len <= SSD1306_BUFFER_SIZE) {
        memcpy(SSD1306_Buffer,buf,len);
        ret = SSD1306_OK;
    }
    return ret;
}

void ssd1306_Bitmap(uint8_t x, uint8_t y, uint8_t *bmp, uint8_t chWidth, uint8_t chHeight)
{
	uint8_t i, j, byteWidth = (chWidth + 7)/8;
	for(j = 0;j < chHeight;j++){
		for(i = 0;i <chWidth;i++){
			if(bmp[j*byteWidth + i/8] & (128 >> (i & 7))){
				ssd1306_DrawPixel(x + i, y + j, White);
			}
		}
	}
}

// Reference commands from Arduino implementation
//	ssd1306_WriteCommand(0xae);//--turn off oled panel
//
//	ssd1306_WriteCommand(0xd5);//--set display clock divide ratio/oscillator frequency
//	ssd1306_WriteCommand(0x80);//--set divide ratio
//
//	ssd1306_WriteCommand(0xa8);//--set multiplex ratio
//	ssd1306_WriteCommand(0x1f);//--1/32 duty
//
//	ssd1306_WriteCommand(0xd3);//-set display offset
//	ssd1306_WriteCommand(0x00);//-not offset
//
//	ssd1306_WriteCommand(0x21);
//	ssd1306_WriteCommand(0x20);
//	ssd1306_WriteCommand(0x20 + (64-1));
//
//	ssd1306_WriteCommand(0x8d);//--set Charge Pump enable/disable
//	ssd1306_WriteCommand(0x14);//--set(0x10) disable
//
//	ssd1306_WriteCommand(0x40);//--set start line address
//
//	ssd1306_WriteCommand(0xa6);//--set normal display
//
//	ssd1306_WriteCommand(0xa4);//Disable Entire Display On
//
//	ssd1306_WriteCommand(0xa1);//--set segment re-map 128 to 0
//
//	ssd1306_WriteCommand(0xC8);//--Set COM Output Scan Direction 64 to 0
//
//	ssd1306_WriteCommand(0xda);//--set com pins hardware configuration
//	ssd1306_WriteCommand(0x12);
//
//	ssd1306_WriteCommand(0x81);//--set contrast control register
//	ssd1306_WriteCommand(0xcf);
//
//	ssd1306_WriteCommand(0xd9);//--set pre-charge period
//	ssd1306_WriteCommand(0xf1);
//
//	ssd1306_WriteCommand(0xdb);//--set vcomh
//	ssd1306_WriteCommand(0x40);
//
//	ssd1306_WriteCommand(0xaf);//--turn on oled panel
//}

// Initialize the oled screen
void ssd1306_Init(I2C_HandleTypeDef *hi2c) {
	SSD1306_I2C_PORT = hi2c;

    // Reset OLED
    ssd1306_Reset();

    // Wait for the screen to boot
    HAL_Delay(100);

    // Init OLED
    ssd1306_SetDisplayOn(0); //display off

    // Mux ratio A8h
    ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
    ssd1306_WriteCommand(0x1F); //
//	ssd1306_WriteCommand(0x3F); //

    // Display offset D3h, 00h
	ssd1306_WriteCommand(0xD3); //-set display offset - CHECK
	ssd1306_WriteCommand(0x00); //-not offset

	// Display start line
	ssd1306_WriteCommand(0x40); //--set start line address - CHECK

	// Segment re-map

    // COM Output scan direction
    ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction

    // COM Pins HW conf
	ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
//	ssd1306_WriteCommand(0x02);
	ssd1306_WriteCommand(0x12);


    // Contrast Control
	ssd1306_SetContrast(0xFF);

    // Disable Entire Display On
	ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    // Set Normal Display
	ssd1306_WriteCommand(0xA6); //--set normal color

    // Set Osc Frequency
	ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(0xF0); //--set divide ratio

    // Enable charge pump regulator
	ssd1306_WriteCommand(0x8D); //--set DC-DC enable
	ssd1306_WriteCommand(0x14); //enable

	ssd1306_WriteCommand(0xDB); //--set vcomh
	ssd1306_WriteCommand(0x20); //0x20,0.77xVcc

    ssd1306_WriteCommand(0xD9); //--set pre-charge period
    ssd1306_WriteCommand(0x22); //


    ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode
    ssd1306_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
    ssd1306_WriteCommand(0x00); //---set low column address
    ssd1306_WriteCommand(0x10); //---set high column address

//	ssd1306_WriteCommand(0x21);
//	ssd1306_WriteCommand(0x20);
//	ssd1306_WriteCommand(0x20 + (SSD1306_WIDTH - 1));

    ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK


	// Display On
    ssd1306_SetDisplayOn(1); //--turn on SSD1306 panel

    // Clear screen
    ssd1306_Fill(Black);

    // Flush buffer to screen
    ssd1306_UpdateScreen();

    // Set default values for screen object
    SSD1306.Initialized = 1;
}

// Fill the whole screen with the given color
void ssd1306_Fill(SSD1306_COLOR color) {
    /* Set memory */
    uint32_t i;

    for(i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

// Write the screenbuffer with changed to the screen
void ssd1306_UpdateScreen(void) {
    // Write data to each page of RAM. Number of pages
    // depends on the screen height:
    //
    //  * 32px   ==  4 pages
    //  * 64px   ==  8 pages
    //  * 128px  ==  16 pages
    for(uint8_t i = 0; i < SSD1306_HEIGHT/8; i++) {
        ssd1306_WriteCommand(0xB0 + i); // Set the current RAM page address.
        ssd1306_WriteCommand(0x00);
        ssd1306_WriteCommand(0x12);
        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH);
    }
}

//    Draw one pixel in the screenbuffer
//    X => X Coordinate
//    Y => Y Coordinate
//    color => Pixel color
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if(SSD1306.Inverted) {
        color = (SSD1306_COLOR)!color;
    }

    // Draw in the right color
    if(color == White) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void ssd1306_SetContrast(const uint8_t value) {
    const uint8_t kSetContrastControlRegister = 0x81;
    ssd1306_WriteCommand(kSetContrastControlRegister);
    ssd1306_WriteCommand(value);
}

void ssd1306_SetDisplayOn(const uint8_t on) {
    uint8_t value;
    if (on) {
        value = 0xAF;   // Display on
        SSD1306.DisplayOn = 1;
    } else {
        value = 0xAE;   // Display off
        SSD1306.DisplayOn = 0;
    }
    ssd1306_WriteCommand(value);
}

uint8_t ssd1306_GetDisplayOn() {
    return SSD1306.DisplayOn;
}

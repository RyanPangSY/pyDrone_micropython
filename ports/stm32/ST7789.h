/**
	******************************************************************************
	* This file is part of the MicroPython project, http://micropython.org/
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.http://bbs.01studio.org/
	* File Name 				 :	ST7789.h
	* Author						 :	Folktale
	* Version 					 :	v1.0
	* date							 :	2021/9/18
	* Description 			 :	
	******************************************************************************
**/

#ifndef MICROPY_INCLUDED_STM32_ST7789_H
#define MICROPY_INCLUDED_STM32_ST7789_H

#include "py/obj.h"

#if (MICROPY_HW_LCD15 && MICROPY_ENABLE_TFTLCD & MICROPY_ENABLE_SPILCD)
	
#include "modtftlcd.h"


extern Graphics_Display st7789_glcd;

extern const mp_obj_type_t ST7789_type;

extern void st7789_set_dir(uint8_t dir);
extern void  mp_init_ST7789(void);
extern uint16_t st7789_readPoint(uint16_t x, uint16_t y);
extern void st7789_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
extern void st7789_Full(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);
extern void st7789_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
extern void st7789_draw_hline(uint16_t x0,uint16_t y0,uint16_t len,uint16_t color);

#endif

#endif // MICROPY_INCLUDED_STM32_ST7789_H

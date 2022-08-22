/* 
 * File:   LCD.h
 * Author: ALBA RODAS
 * Comments: COPIA PARA LAB#4
 * Revision history: 
 */

/* 
 * File: LCD8.h  
 * Se utiliz� y se adaptaron las librer�as de Ligo George 
 * de la p�gina www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */
 
#ifndef LCD_H
#define	LCD_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 1000000
#endif

#ifndef RS
#define RS RC6
#endif

#ifndef EN
#define EN RC7
#endif

#include <xc.h> 
//LCD Functions Developed by electroSome

void Lcd_Port(char a);

void Lcd_Cmd(char a);

void Lcd_Clear(void);

void Lcd_Set_Cursor(char a, char b);

void Lcd_Init(void);

void Lcd_Write_Char(char a);

void Lcd_Write_String(char *a);

void Lcd_Shift_Right(void);

void Lcd_Shift_Left(void);

#endif
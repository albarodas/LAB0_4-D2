/*
 * File:   LCD.c
 * Author: ALBA RODAS
 *
 * Created on 29 de julio de 2022, 08:21 AM
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <pic16f887.h>
#include "LCD.h"

#define _XTAL_FREQ 8000000

void Lcd_Port(char a) 
{
    PORTD = a;
}

void Lcd_Cmd(char a) 
{
    RS = 0;
    PORTD = a;
    EN = 1;
    __delay_ms(4);
    EN = 0;
    
}

void Lcd_Clear(void) { //Limpieza
    Lcd_Cmd(0);
    Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b) //Establecimiento de posición
{
    char x;
    if (a == 1) { //Si es fila 1, se establece col.
        x = 0x80 + b - 1;
        Lcd_Cmd(x);
    }
    else if (a == 2) { //Si es fila 2, se establece col.
        x = 0xC0 + b - 1;
        Lcd_Cmd(x);
    }
}

void Lcd_Init(void) {

    Lcd_Cmd(0x00); // Se inicializa en 0
    __delay_ms(20);
    Lcd_Cmd(0x30); // Funcionamiento de 8 bits
    __delay_ms(5);
    Lcd_Cmd(0x30); // Funcionamiento de 8 bits
    __delay_ms(10);     
    Lcd_Cmd(0x30); // Funcionamiento de 8 bits
    __delay_us(100);
//-----------------------------------------------------
    Lcd_Cmd(0x38); // Funcionamiento de 8 bits con 2 lineas y 5x8 pixeles
    Lcd_Cmd(0x0C); // Visualizador activado y apagado, cursor apagado y parpadeo apagado
    Lcd_Cmd(0x01); // Se borra el visualizador 
    Lcd_Cmd(0x06); // Modo de entrada de incremento encendido y desplazamiento apagado
}

void Lcd_Write_Char(char a) {

    RS = 1;
    PORTD = a;;
    EN = 1;
    __delay_us(40);
    EN = 0;

}

void Lcd_Write_String(char *a) {
    char i;
    for (i = 0; a[i] != '\0'; i++) //Fin de cadena
        Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right(void) {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left(void) {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x08);
}
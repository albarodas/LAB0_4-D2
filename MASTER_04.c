/*
 * File:   MASTER_04.c
 * Author: ALBA RODAS
 * Created on 21 de agosto de 2022, 06:57 AM
 */
/*===============================================================================
 * INCLUIMOS LAS LIBRERIAS ".h" A UTILIZAR:
================================================================================*/
#include <xc.h>
#include <stdint.h>
#include <stdio.h> 
#include <pic16f887.h>
#include "I2C.h"
#include "OSCILADOR.h"
#include "LCD.h"

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIGURATION WORDS: 1
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIGURATION WORDS: 2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
//-------------------------------------------------------------------------------
// DECLARAMOS FREQ. DE OSCILADOR:
#define _XTAL_FREQ 1000000
//---------------------------------------------------------------------------
// MAPPING VALORES DE ENTRADA:
#define IN_MIN 0
#define IN_MAX 255
// MAPPING VALORES DE SALIDA:
#define OUT_MIN 0
#define OUT_MAX 500
//-------------------------------------------------------------------------------
//===============================================================================
// VARIABLES:
//===============================================================================
//-------------------------------------------------------------------------------
// VARIABLES PARA POTENCIOMETROS:
uint8_t POT = 0;
// VARIABLES DE TIEMPO
uint8_t seg = 0;
uint8_t min = 0;
uint8_t horas = 0;
// VARIABLES PARA STORE EL VALOR DE VOLTAJE EN NO. ENTEROS, DECIMALES.
uint8_t V_01 = 0;
uint8_t V_02 = 0;
// VARIABLE PARA GUARDAR EL RESULTADO DEL MAPEO:
uint16_t VOLTAJE_01 = 0;
// DIVISION DE UNIDADES DE TIEMPO:
// UNIDADES - DECENAS DE SEGUNDOS:
char unidades_01 = 0;
char decenas_01 = 0;
// UNIDADES - DECENAS DE MINUTOS:
char unidades_02 = 0;
char decenas_02 = 0;
// UNIDADES - DECENAS DE HORAS:
char unidades_03 = 0;
char decenas_03 = 0;
/////////////////////////////////////////////////////////////////////////////////
// VARIABLES PARA MAPEO DE VOLTAJE (LISTADO):
char VOLTAJE_L01[20];       // SIMILAR AL USADO EN LAB#2
/*===============================================================================
 * PROTOTIPO DE FUNCIONES:
================================================================================*/
int traduccion_01 (int a);
int traduccion_02 (int a);
// PROTOTIPO PARA MAPEO VISTO EN PROG. MICROCONTROLADORES 1:
unsigned short map(uint16_t val, uint16_t in_min, uint16_t in_max, unsigned short out_min, unsigned short out_max);
void setup(void);
/*===============================================================================
 * CICLO PRINCIPAL:
================================================================================*/
void main(void)
{
    setup(); 
    while(1)
    {
        // SOLICITUD DE DATOS AL SENSOR:
        // INICIAMOS LA COMUNICACION I2C (LIBRERÍA):
        I2C_Master_Start();         
        // ESTABLECEMOS SI SE ENVIARAN O RECIBIRAN DATOS Y EL ADDRESS:
        I2C_Master_Write(0xD0);  
        // ESCRIBIMOS = 0:
        I2C_Master_Write(0); 
        // DETENEMOS EL PROCESO:
        I2C_Master_Stop(); 
        // DAMOS UN DELAY PARA PROCESAR:
        __delay_ms(10);
        
    // *****************************************************************
        // PROCESO DE LECTURA DEL SENSOR:
        // INICIAMOS LA COMUNICACION I2C (LIBRERÍA):
        I2C_Master_Start();         
        // ESTABLECEMOS SI SE ENVIARAN O RECIBIRAN DATOS Y EL ADDRESS + 1:
        I2C_Master_Write(0xD1);   
        // LEEMOS Y ENVIAMOS EL "OK" = ACKNOWLEDGE = ACK.
        // ESTO PARA CADA UNIDAD DE TIEMPO.
        // APROVECHAR PARA REALIZAR LA CONVERSIÓN:
        seg = traduccion_01(I2C_Master_Read(1)); 
        min = traduccion_01(I2C_Master_Read(1)); 
        horas = traduccion_01(I2C_Master_Read(1)); 
        // DETENEMOS EL PROCESO:
        I2C_Master_Stop();  
        // DAMOS UN DELAY PARA PROCESAR:
        __delay_ms(10);
        
    // ********************************************************************
        // COMUNICACION I2C CON POT:
        // INICIAMOS LA COMUNICACION I2C (LIBRERÍA):
        I2C_Master_Start();       
        // ESTABLECEMOS SI SE ENVIARAN O RECIBIRAN DATOS Y EL ADDRESS + 1:
        I2C_Master_Write(0xD1);     
        // LEEMOS Y ENVIAMOS EL "OK" = ACKNOWLEDGE = ACK.
        I2C_Master_Read(1); 
        // DETENEMOS EL PROCESO:
        I2C_Master_Stop();          
        __delay_ms(10);
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        // OBTENCION DE UNIDADES, DECENAS, CENTENAS DE UNIDAD DE TIEMPO:
        // SEGUNDOS:
        unidades_01 = seg%10; 
        decenas_01 = seg/10; 
        // MINUTOS:
        unidades_02 = min%10; 
        decenas_02 = min/10; 
        // HORAS:
        unidades_03 = horas&10; 
        decenas_03 = horas/10; 
        //////////////////////////////////////////////////////////////////
        // UTILIZAMOS COMANDOS DE LAS LIBRERIAS DADAS DEL LDC:
        // DEFINIMOS POSICIÓN DE CURSOR:
        Lcd_Set_Cursor(2,8);
        // IMPRIMIMOS LOS VALORES DE CADA UNA DE LAS VARIABLES ANTERIORES:
        // BUSCAMOS QUE SE VEA ASÍ : 00:00:00 --> HORAS:MINUTOS:SEGUNDOS
        Lcd_Write_Char(decenas_03+'0');
        Lcd_Write_Char(unidades_03+'0');
        Lcd_Write_Char(':');
        Lcd_Write_Char(decenas_02+'0');
        Lcd_Write_Char(unidades_02+'0');
        Lcd_Write_Char(':');
        Lcd_Write_Char(decenas_01+'0');
        Lcd_Write_Char(unidades_01+'0');
        
        // ESCRIBIMOS EN MASTER PARA EL ADRESH DEL POT:
        // INICIO:
        I2C_Master_Start();    
        // LECTURA Y ENVÍO:
        I2C_Master_Write(0x71);     
        // ACK:
        POT = I2C_Master_Read(0); 
        // STOP:
        I2C_Master_Stop();         
        __delay_ms(10);
        ////////////////////////////////////////////////////////
        // MAPPING DEL VOLTAJE --> TENDRÉ QUE CONCATENAR MÁS ADELANTE.
        VOLTAJE_01 =(uint16_t)(map(POT, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX));
        // PARA VOLTAJE ENTERO:
        V_01 = VOLTAJE_01/100; 
        // PARA VOLTAJE EN DECIMAL:
        V_02 = VOLTAJE_01 - V_01*100; 
        ////////////////////////////////////////////////////
        // UBICAMOS EL CURSOR EN POSICION QUE NO INTERFIERA CON SENSOR:
        Lcd_Set_Cursor(3,8);
        // PARTE DE CONCATENAR MENCIONADA:
        sprintf(VOLTAJE_L01, "%d.%2dV ", V_01, V_02); 
        Lcd_Set_Cursor(2,1); 
        // IMPRIMO LO CONCATENADO EN FORMA DE STRING:
        Lcd_Write_String(VOLTAJE_L01); 
        // START:
            I2C_Master_Start();        
            // ENVÍO --> WRITE:
            I2C_Master_Write(0x70);   
            // ESCRIBO:
            I2C_Master_Write(PORTB); 
            // STOP:
            I2C_Master_Stop();         
            __delay_ms(100);
    }
    return;
}
void setup(void){
    
    // CONFIG. RELOJ INTERNO PIC.
    initOsc(1); // OSCILADOR --> 1MHz.
    // FRECUENCIA --> 1MHZ --> MEJOR FUNCIONAMIENTO DE LA LCD. 
    // HABILITAMOS PINES ANALOGICOS, SI HUBIERAN:
    ANSEL = 0; 
    ANSELH = 0; 
    TRISB = 0xFF; 
    // PULLUP INTERNOS DE PORTB --> RB0 - RB3 --> 4 BOTONES POR LED.
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0b1111; 
    // SALIDA PARA LCD, R y EN:
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    // EN EL PUERTO D, SE CONECTAN LOS PINES DE LA LCD, POR ENDE SON SALIDAS:
    TRISD = 0; 
    PORTD = 0; 
    // INICIALIZAMOS LA COMUNICACION I2C:
    // ARGUMENTO = VALOR DEL CLK A UTILIZAR --> 100000kHz.
    I2C_Master_Init(100000); 
    // USAMOS LA LCD EN MODALIDAD 8BITS, PARA MODIFICACIONES (16BITS) --> LCD.h, LCD.c
    Lcd_Init();
    // LIMPIAMOS PARA INICIAR CON VALORES ALEATORIOS.
    Lcd_Clear();  
    // DEFINIMOS EL ENCABEZADO PARA LA LCD: POTENCIOMETRO Y TIMER.
    Lcd_Write_String(" POT:    TIME:");  // Mensaje a desplegar
}
/*
 * FORMA DE TRABAJO DE LA TRADUCCION_01:
 * BCD = (binary-coded decimal).
 * Converts input unsigned short integer number to its appropriate BCD representation.
    unsigned short a, b;
    ...
    a = 22;
    b = Dec2Bcd(a);    // b = 34
 */
int traduccion_01 (int a)
{ 
    return (a >> 4) * 10 + (a & 0x0F);
}
/*
 * FORMA DE TRABAJO DE LA TRADUCCION_02:
 * BCD = (binary-coded decimal).
 * Converts 8-bit BCD numeral to its decimal equivalent.
    unsigned short a, b;
    ...
    a = 0x34;          // a = 0x34
    b = Bcd2Dec(a);    // b = 22
 */
int traduccion_02 (int a)
{
    return ((a / 10) << 4) + (a % 10);
}
// MAPEO, LO PONGO AQUI PORQUE ME DIO GRAN ERROR PONIENDOLO ARRIBA:
unsigned short map(uint16_t x, uint16_t x0, uint16_t x1, unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
/*
 * File:   ESCLAVO_04.c
 * Author: ALBA RODAS
 * Created on 21 de agosto de 2022, 07:11 AM
 */

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/*===============================================================================
 * INCLUIMOS LAS LIBRERIAS ".h" A UTILIZAR:
================================================================================*/
#include <xc.h>
#include <stdint.h>
#include <stdio.h> 
#include <pic16f887.h>
#include "I2C.h"
#include "OSCILADOR.h"
#include "ADC.h"
//-------------------------------------------------------------------------------
// DECLARAMOS FREQ. DE OSCILADOR:
#define _XTAL_FREQ 1000000      // VALOR DE OSC: 1MHz
//-------------------------------------------------------------------------------
// CONFIGURATION WORDS: 1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
//-------------------------------------------------------------------------------
//===============================================================================
// VARIABLES:
//===============================================================================
//-------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// VARIABLES PARA BUFFER:
uint8_t BUFFER = 0x00;
// VARIABLES PARA Z, SHOW VALUES :
short impedancia_z = 0;
// VARIABLES PARA POTENCIOMETROS:
uint8_t POT = 0;
//------------------------------------------------------------------------------
void __interrupt() isr(void)
{
    // LECTURA DEL ESCLAVO --> I2C
   if(PIR1bits.SSPIF == 1)
   { 
        SSPCONbits.CKP = 0;
        
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL))
        {
            // SI "Z" == VALOR DEL BUFFER 
            // I2C MASTER MODE WAVEFORM:
            // SSPBUF = Synchronous Serial Port Receive Buffer/Transmit Register
            impedancia_z = SSPBUF;
            // SSPOV = 1, CUANDO EL SSPBUF ESTÁ AÚN FULL.
            // SSPOV = Receive Overflow Indicator bit --> SOLO OCURRE EN MODO ESCLAVO
            // NO HAY OVERFLOW, ENTONCES SE MANTIENE = 0.
            SSPCONbits.SSPOV = 0;
            // WCOL: Write Collision Detect bit.
            // SUCEDE EN MASTER Y SLAVE MODE.
            // Slave mode --> WCOL = 0 --> NO HAY DATOS EN EL BUFFER AÚN.
            SSPCONbits.WCOL = 0;
            // CKP: Clock Polarity Select bit.
            // CKP --> FUNCIONA EN MODO SPI e I2C.
            // I2C --> SCK release control --> 1 = Enable clock 
            // ------------------------------> 0 = 0 = Holds clock low (clock stretch). (Used to ensure data setup time.)
            SSPCONbits.CKP = 1;
            // ACTIVAMOS NUEVAMENTE EL SCK.
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) //Operación de lectura
        {
            //Slave address AND Multi-byte read
            // SI "Z" == VALOR DEL BUFFER 
            // I2C MASTER MODE WAVEFORM:
            // SSPBUF = Synchronous Serial Port Receive Buffer/Transmit Register
            // Read the previous value to clear the buffer
            impedancia_z = SSPBUF;
            // SSPIF: Master Synchronous Serial Port (MSSP) Interrupt Flag bit.
            // CLR A LA BANDERA DEL SSPIF
            PIR1bits.SSPIF = 0; 
            // I2C --> SCK release control --> 1 = Enable clock 
            SSPCONbits.CKP = 1;
            
            // MIENTRAS EL BUFFER NO ESTÉ SET, WAIT 250us.
            while(!SSPSTATbits.BF);
            BUFFER = SSPBUF; // Se almacena valor enviado por controlador
            __delay_us(200);
            
        }
        // DE LO CONTRARIO EL D_not_A y R_not_W NO ESTÉ SET:
        // ESCRITURA EN BUFFER:
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW) //Operación de escritura
        {
            // Read the previous value to clear the buffer
            impedancia_z = SSPBUF;
            // CLR EL BUFFER.
            SSPSTATbits.BF = 0;
            // SEND EL ADRESH PROVENIENTE DEL POTENCIOMETRO EN RA0.
            SSPBUF = POT; 
            // I2C --> SCK release control --> 1 = Enable clock 
            SSPCONbits.CKP = 1;
            __delay_us(200);
            
            while(SSPSTATbits.BF);
        }
        // SSPIF: Master Synchronous Serial Port (MSSP) Interrupt Flag bit.
        // CLR A LA BANDERA DEL SSPIF
        PIR1bits.SSPIF = 0;    
    }
   // IMPLEMENTACIÓN DEL ADC PARA CAMBIO DE VALORES DEL POTENCIOMETRO:
   if(PIR1bits.ADIF)
   {                        
       // IF VALUE CHANGES, CORRIMIENTO DE BITS, USANDO LIBRERIA.
        if(ADCON0bits.CHS == 0){ // Si varía potenciómetro
            POT = (adc_read()>>2);
            // SI PONGO CORRIMIENTO DE >>1 --> ALCANZA EL 50% Y REINICIA CONTEO 
            // HASTA NUEVAMENTE ALCANZAR 0V o 5V.
        }
        // LIMPIAMOS BANDERA DEL ADCE
        PIR1bits.ADIF = 0; 
    }
}

void main(void)
{
    // CONFIGURACIONES:
    // OSCILADOR --> 1MHz, FUNCIONA MEJOR LA LCD:
    initOsc(1);
    // SET ENTRADA ANALOGICA PARA EL POT:
    ANSEL = 0b01;  
    ANSELH = 0; 
    // SET ENTRADA ANALOGICA PARA EL POT:
    TRISA = 0b01;
    // CLR A PUERTOS:
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    // TRIS:
    TRISB = 0;
    TRISD = 0;
    // LLAMAMOS A LAS FUNCIONES EN ADC.c y ADC.h
    adc_init(0, 0, 0); 
    // SET DIRECCIÓN DE COMUNICACIÓN --> RECEPCIÓN DE DATOS PARA LEDS.
    I2C_Slave_Init(0x70); //Se inicia I2C Slave con address 0xA0
    
    while(1)
    {
        // USAMOS CANAL 0, POR ENDE INICIA LA CONVERSIÓN:
        adc_start(0); 
        
        // SECCIÓN PARA DEFINIR ENCENDIDO O APAGADO DE LEDS:
        if(BUFFER&1)
        {
            // COMO UN SWITCH CASE, HACEMOS & CON EL BUFFER.
            PORTBbits.RB0 = 0;
            __delay_ms(10);
            
        } else 
        {
            // SI EL RESULTADO NO ES == 1, SE ENCIENDE EL LED.
            PORTBbits.RB0 = 1;
            __delay_ms(10);
        }
        if(BUFFER&2)
        {
            // COMO UN SWITCH CASE, HACEMOS & CON EL BUFFER.
            PORTBbits.RB1 = 0;
            __delay_ms(10);
            
        } 
        else 
        {
            // SI EL RESULTADO NO ES == 1, SE ENCIENDE EL LED.
            PORTBbits.RB1 = 1;
            __delay_ms(10);
        }
        if(BUFFER&4)
        {
            // COMO UN SWITCH CASE, HACEMOS & CON EL BUFFER.
            PORTBbits.RB2 = 0;
            __delay_ms(10);
            
        } 
        else 
        {
             // SI EL RESULTADO NO ES == 1, SE ENCIENDE EL LED.
            PORTBbits.RB2 = 1;
            __delay_ms(10);
        }
        if(BUFFER&8)
        {
            // COMO UN SWITCH CASE, HACEMOS & CON EL BUFFER.
            PORTBbits.RB3 = 0;
            __delay_ms(10);
            
        } 
        else 
        { 
            // SI EL RESULTADO NO ES == 1, SE ENCIENDE EL LED.
            PORTBbits.RB3 = 1;
            __delay_ms(10);
        } 
    }
    return;
}
// END.
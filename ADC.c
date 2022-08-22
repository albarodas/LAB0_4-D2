/* 
 * File:   ADC.c
 * Author: ALBA RODAS
 * Comments: COPIA PARA LAB#4
 * Revision history: 
 */

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 1000000 
#endif

#include <xc.h>
#include "ADC.h"

void adc_init(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus)
{
    switch(adc_cs)
    {
        case 0: 
            // FOSC/2:
            ADCON0bits.ADCS = 0b00;
            break;
        case 1: 
            // FOSC/8:
            ADCON0bits.ADCS = 0b01;
            break;
        case 2: 
            // FOSC/32:
            ADCON0bits.ADCS = 0b10;
            break;
        case 3: 
            // F_RC:
            ADCON0bits.ADCS = 0b11;
            break;
        default: 
            // FOSC/8:
            ADCON0bits.ADCS = 0b01;
            break;
    }
    
    switch(vref_plus)
    {
        case 0: 
            // V_DD:
            ADCON1bits.VCFG0 = 0;
            break;
        case 1: 
            // V_REF:
            ADCON1bits.VCFG0 = 1;
            break;
        default: 
            ADCON1bits.VCFG0 = 0;
            break;
    } 
    switch(vref_minus)
    {
        case 0: 
            // V_SS:
            ADCON1bits.VCFG1 = 0;
            break;
        case 1: 
            // V_REF:
            ADCON1bits.VCFG1 = 1;
            break;
        default: 
            // V_SS:
            ADCON1bits.VCFG1 = 0;
            break;
    }
    ADCON1bits.ADFM = 1; // JUSTIFICACION A LA IZQUIERDA.
    ADCON0bits.ADON = 1; // ADC = ON.
    PIE1bits.ADIE = 1;   // INTERRUPCIONES DEL ACD = ON.
    PIR1bits.ADIF = 0;   // CLR A LA BANDERA DEL ADC.
}
void adc_start(uint8_t channel)
{
    if(ADCON0bits.GO == 0){  
        // CHECK IF CONVERSION:
        ADCON0bits.CHS = channel; 
        // SELECT UN CANAL:
        __delay_us(40); 
        // DELAY MÍNIMO DE LECTURA >= 40us
        ADCON0bits.GO = 1; 
        // START CONVERSION.
    }    
}
uint16_t adc_read(void)
{
    PIR1bits.ADIF = 0; 
    // CLR AL ADCE¿:
    return (uint16_t)((ADRESH << 8) + ADRESL);
    // ADRESH:ADRESL VUELVE A LOS 10 BITS ORIGINALES.
}
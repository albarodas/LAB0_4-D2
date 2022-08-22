/* 
 * File:   OSCILADOR.c
 * Author: ALBA RODAS
 * Comments: COPIA PARA LAB#4
 * Revision history: 
 */

#include <stdint.h>
#include "OSCILADOR.h"

void initOsc(uint8_t freq){
    switch(freq){
        case 1:
            OSCCONbits.IRCF = 0b100;
            // FOSC = 1 MHZ
            break;
        case 2:
            OSCCONbits.IRCF = 0b101; 
            // FOSC = 2 MHz
            break;
        case 4:
            OSCCONbits.IRCF = 0b110; 
            // FOSC = 4 MHz
            break;
        case 8:
            OSCCONbits.IRCF = 0b111; 
            // FOSC = 8 MHz
            break;
        default: 
            OSCCONbits.IRCF = 0b110; 
            // FOSC = 4 MHz
            break;
    }
    // ADICIONAL:
    OSCCONbits.SCS = 1;
}

    
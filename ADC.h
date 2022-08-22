/* 
 * File:   ADC.h
 * Author: ALBA RODAS
 * Comments: COPIA PARA LAB#4
 * Revision history: 
 */

#ifndef ADC_H
#define	ADC_H

#include <xc.h>
#include <stdint.h>

void adc_init(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);

void adc_start(uint8_t channel);
/*
 * CANALES DISPONIBLES: 1 - 15
 */
uint16_t adc_read(void);
/*
 * CLR BANDERA.
 */
#endif	
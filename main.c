//*****************************************************************************
// Universidad del Valle de Guatemala
// Programación de Microcontroladores
// Archivo: Proyecto
// Hardware: ATMEGA328P
// Autor: LISBETH ARÉVALO
// Carnet: 22757
//*****************************************************************************

#define F_CPU 16000000UL // Define la frecuencia de la CPU
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "PWM/PWM0.h"
#include "PWM/PWM1.h"
#include "PWM/PWM2.h"
#include "ADC/ADC.h"

// Variables globales para almacenar los ciclos de trabajo del PWM
uint8_t CicloUtil1 = 0;
uint8_t CicloUtil2 = 0;
uint8_t CicloUtil3 = 0;
uint8_t CicloUtil4 = 0;
volatile int Modo = 0; // Variable global para el estado del sistema

// Declaración de funciones
void inicializar(void);
unsigned char leer_eeprom(unsigned int direccion); // Declaración de la función leer_eeprom
void escribir_eeprom(unsigned int direccion, unsigned char dato); // Declaración de la función escribir_eeprom

int main(void) {
    cli(); // Desactiva las interrupciones globales

    // Configura PD6, PD7, PD4 y PD2 como entradas y habilita los pull-ups internos
    DDRD &= ~((1 << DDD6) | (1 << DDD7) | (1 << DDD4) | (1 << DDD2));
    PORTD |= (1 << PORTD6) | (1 << PORTD7) | (1 << PORTD4) | (1 << PORTD2);
    
    // Configura PB0, PB4 y PB5 como entradas y habilita los pull-ups internos
    DDRB &= ~((1 << DDB0) | (1 << DDB4) | (1 << DDB5));
    PORTB |= (1 << PORTB0) | (1 << PORTB4) | (1 << PORTB5);
    
    // Configura PC0, PC1 y PC2 como salidas y los inicializa en bajo
    DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2);
    PORTC &= ~((1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2));

    // Inicializa ADC y PWM
    iniciarADC();
    iniciarPWM1A(no_invertido, 8, 39999);
    iniciarPWM1B(no_invertido, 8, 39999);
    iniciarPWM2A(no_invertido, 1024);
    iniciarPWM2B(no_invertido, 1024);

    // Habilita las interrupciones de cambio de pin para PCINT23, PCINT20 y PCINT18
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT23) | (1 << PCINT20) | (1 << PCINT18);

    // Habilita las interrupciones de cambio de pin para PCINT0, PCINT4 y PCINT5
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT4) | (1 << PCINT5);

    UCSR0B = 0; // Deshabilita TX y RX

    sei(); // Activa las interrupciones globales

    while (1) {
        // Máquina de estados
        switch (Modo) {
            case 0:
                PORTC = (1 << PC0); // Enciende el LED en PC0
                _delay_ms(10);
                
                // Lee los valores de ADC y actualiza los ciclos de trabajo del PWM
                CicloUtil1 = ADC_CONVERT(6);
                actualizarCicloUtilA1(CicloUtil1);

                _delay_ms(10);
                CicloUtil2 = ADC_CONVERT(4);
                actualizarCicloUtilA2(CicloUtil2);

                _delay_ms(10);
                CicloUtil3 = ADC_CONVERT(5);
                actualizarCicloUtilB1(CicloUtil3);

                _delay_ms(10);
                CicloUtil4 = ADC_CONVERT(7);
                actualizarCicloUtilB2(CicloUtil4);

                break;

            case 1:
                PORTC = (1 << PC1); // Enciende el LED en PC1
                break;

            case 2:
                PORTC = (1 << PC2); // Enciende el LED en PC2
                break;

            default:
                Modo = 0; // Reinicia el estado si es desconocido
                break;
        }
    }
}

// ISR para las interrupciones de cambio de pin en el puerto D
ISR(PCINT2_vect) {
    // Si el botón conectado a PD7 es presionado
    if ((PIND & (1 << PIND7)) == 0) {
        _delay_ms(50); // Debounce
        if ((PIND & (1 << PIND7)) == 0) {
            Modo++;
            if (Modo > 2) {
                Modo = 0; // Reinicia el estado si sobrepasa el límite
            }
        }
    }
    // Si el botón conectado a PD4 es presionado
    else if ((PIND & (1 << PIND4)) == 0) {
        PORTC = (1 << PC2); // Enciende el LED en PC2

        if (Modo == 0) {
            // Guarda los valores de los ciclos de trabajo en la EEPROM
            escribir_eeprom(0, CicloUtil1);
            escribir_eeprom(1, CicloUtil2);
            escribir_eeprom(2, CicloUtil3);
            escribir_eeprom(3, CicloUtil4);
        } else if (Modo == 1) {
            // Lee los valores de los ciclos de trabajo desde la EEPROM
            uint8_t leerCicloUtil1 = leer_eeprom(0);
            actualizarCicloUtilA1(leerCicloUtil1);
            uint8_t leerCicloUtil2 = leer_eeprom(1);
            actualizarCicloUtilA2(leerCicloUtil2);
            uint8_t leerCicloUtil3 = leer_eeprom(2);
            actualizarCicloUtilB1(leerCicloUtil3);
            uint8_t leerCicloUtil4 = leer_eeprom(3);
            actualizarCicloUtilB2(leerCicloUtil4);
        }
    }
    // Si el botón conectado a PD2 es presionado
    else if ((PIND & (1 << PIND2)) == 0) {
        if (Modo == 0) {
            PORTC = (1 << PC2); // Enciende el LED en PC2

            // Guarda los valores de los ciclos de trabajo en la EEPROM
            escribir_eeprom(12, CicloUtil1);
            escribir_eeprom(13, CicloUtil2);
            escribir_eeprom(14, CicloUtil3);
            escribir_eeprom(15, CicloUtil4);
        } else if (Modo == 1) {
            PORTC = (1 << PC2); // Enciende el LED en PC2

            // Lee los valores de los ciclos de trabajo desde la EEPROM
            uint8_t leerCicloUtil1 = leer_eeprom(12);
            actualizarCicloUtilA1(leerCicloUtil1);
            uint8_t leerCicloUtil2 = leer_eeprom(13);
            actualizarCicloUtilA2(leerCicloUtil2);
            uint8_t leerCicloUtil3 = leer_eeprom(14);
            actualizarCicloUtilB1(leerCicloUtil3);
            uint8_t leerCicloUtil4 = leer_eeprom(15);
            actualizarCicloUtilB2(leerCicloUtil4);
        }
    }
}

// ISR para las interrupciones de cambio de pin en el puerto B
ISR(PCINT0_vect) {
    // Si el botón conectado a PB0 es presionado
    if ((PINB & (1 << PINB0)) == 0) {
        PORTC = (1 << PC2); // Enciende el LED en PC2

        if (Modo == 0) {
            // Guarda los valores de los ciclos de trabajo en la EEPROM
            escribir_eeprom(4, CicloUtil1);
            escribir_eeprom(5, CicloUtil2);
            escribir_eeprom(6, CicloUtil3);
            escribir_eeprom(7, CicloUtil4);
        } else if (Modo == 1) {
            // Lee los valores de los ciclos de trabajo desde la EEPROM
            uint8_t leerCicloUtil1 = leer_eeprom(4);
            actualizarCicloUtilA1(leerCicloUtil1);
            uint8_t leerCicloUtil2 = leer_eeprom(5);
            actualizarCicloUtilA2(leerCicloUtil2);
            uint8_t leerCicloUtil3 = leer_eeprom(6);
            actualizarCicloUtilB1(leerCicloUtil3);
            uint8_t leerCicloUtil4 = leer_eeprom(7);
            actualizarCicloUtilB2(leerCicloUtil4);
        }
    }
    // Si el botón conectado a PB4 es presionado
    else if ((PINB & (1 << PINB4)) == 0) {

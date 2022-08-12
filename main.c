
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*------------------------------------------------------------------------------
 * LIBRERIAS
 ------------------------------------------------------------------------------*/

#include <xc.h>
#include "SPI.h"
#include "PWM.h"
#include "LCD.h"
#include "OSCILADOR.h"
#include "TMR0.h"

#include <xc.h>
#include <stdio.h>
#include <stdint.h>

#include <pic16f887.h>



#define _XTAL_FREQ 4000000



/*------------------------------------------------------------------------------
 * Constantes
 ------------------------------------------------------------------------------*/
uint8_t bandera = 0;
uint8_t valor_pot1 = 0;
uint8_t valor_pot2 = 0;
uint8_t valor_ADC = 0;
uint8_t potenciometro1 = 0;
uint8_t potenciometro2 = 0;


/*------------------------------------------------------------------------------
 * Codigo
 ------------------------------------------------------------------------------*/
void setup(void);

void __interrupt() isr (void){
   
        if(PIR1bits.SSPIF){                  // int. SPI
            if (bandera == 1){              // enviamos potenciometro
                spiWrite(valor_pot1);
                bandera = 0;
            }
            else if (bandera == 0){         // enviamos potenciometro1
                spiWrite(valor_pot2);
                bandera = 1;
            }
            PIR1bits.SSPIF = 0;             // limpiamos bandera de interrupción
        }
        else if(PIR1bits.ADIF){
            if(valor_ADC == 0){
                potenciometro1 = adc_read();          // leemos pot1
            }
            else if(valor_ADC == 1){
                potenciometro2 = adc_read();          // leemos pot2
            }
        }
    
    return;
}



void main(void){
    setup();
    while(1){
        if (valor_ADC == 0){                // canal 0
            adc_start(0);
            valor_ADC = 1;
        }
        else if (valor_ADC == 1){           // canal 1
            adc_start(1);
            valor_ADC = 0;
        }
        valor_pot1 = potenciometro1 & 0b11111110;     // indicador de potenciómetros
        valor_pot2 = potenciometro2 | 0b00000001;     
    }
}

void setup(void){
    TRISD = 0x00;
    TRISC = 0b00011000;
    TRISA = 0b00100000
    ANSEL = 0b00000011;
    TRISAbits.TRISA5 = 1;
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    TRISC = 0b000011000;
    PORTC = 0;
    int_osc_MHz(4); // 0 ---> 1MHz, 1 ---> 2MHz, 2 ---> 4MHz, 3 ---> 8MHz, 4 ---> 500kHz, default ---> 4MHz
    adc_init(1,0,0);
    tmr0_init (256);
    
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_END, SPI_CLOCK_IDLE_HIGH, SPI_IDLE_2_ACTIVE);
    
    PIE1bits.RCIE = 1;
    INTCONbits.T0IE = 1; //Habiliatamos int. TMR0
    tmr0_reload();   //función de TMR0
    tmr0_init(256);  //configuración prescaler 256 TMR0
    PIE1bits.SSPIE = 1;
    PIE1bits.ADIE = 1;
    
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
}

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

#include <stdio.h>
#include <stdint.h>

#include <pic16f887.h>






/*------------------------------------------------------------------------------
 * Constantes
 ------------------------------------------------------------------------------*/

#define _XTAL_FREQ 4000000
#define FLAG_SPI 0x0F

uint8_t valor_potenciometro = 0;
uint8_t bandera = 0;
uint8_t pot1 = 0;
uint8_t pot2 = 0;
uint8_t entero1 = 0;
uint8_t entero2 = 0;
uint8_t dec1 = 0;
uint8_t dec2 = 0;

unsigned short voltaje_1 = 0;
unsigned short voltaje_2 = 0;
char s[];

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}


void setup(void);

void main(void){
    setup();
    Lcd_Clear();
    Lcd_Set_Cursor(1,1); // primera fila
    Lcd_Write_String("  POT1    POT2");// escribir en la lcd
    
    
    while(1){
        spiWrite(FLAG_SPI); // valor para generar respuesta del slave
        spiReceiveWait(); // se espera envío

        PORTCbits.RC2 = 1;  // deshabilitamos slave
        __delay_ms(10);  
        PORTCbits.RC2 = 0;  // habilitamos slave
        __delay_ms(10);

        spiReceiveWait(); 
        valor_potenciometro = spiRead();    // guardamos lo recibido
        PORTB = valor_potenciometro;
        
        __delay_ms(10);
        
        bandera = 0b00000001 & valor_potenciometro;
        if (bandera == 1){
            pot1 = valor_potenciometro;       //Guardamos pot1 
            
            voltaje_1 = map(pot1, 1, 255, 0, 500); // Enteros y decimales de pot1
            entero1 = voltaje_1/100;
            dec1 = voltaje_1-entero1*100;
        }
        else if (bandera == 0) {
            pot2 = valor_potenciometro;      //Guardamos pot2 
            
            voltaje_2 = map(pot2, 1, 255, 0, 500);// Enteros y decimales de pot2
            entero2 = voltaje_2/100;
            dec2 = voltaje_2-entero2*100;
        }

        sprintf(s, "  %d.%d    %d.%d ", entero1, dec1, entero2, dec2); // Cadena de valores
        Lcd_Set_Cursor(2,1); // segunda fila
        Lcd_Write_String(s); // escribir en la lcd

    }
    
}

    
    

void setup(void){
    TRISD = 0x00;
    TRISC = 0b00010001;
    PORTC = 0;
    TRISAbits.TRISA5 = 0; 
    PORTA = 1;
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    int_osc_MHz(4); // 0 ---> 1MHz, 1 ---> 2MHz, 2 ---> 4MHz, 3 ---> 8MHz, 4 ---> 500kHz, default ---> 4MHz
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    Lcd_Init();
    
    TRISE = 0;
    PORTE = 0;
    
    //PIE1bits.RCIE = 1;
    //INTCONbits.T0IE = 1; //Habiliatamos int. TMR0
    //tmr0_reload();   //función de TMR0
    //tmr0_init(256);  //configuración prescaler 256 TMR0
    //PIE1bits.SSPIE = 1;
    //PIE1bits.ADIE = 1;
    
    //PIR1bits.SSPIF = 0;
    
    
    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_END, SPI_CLOCK_IDLE_HIGH, SPI_IDLE_2_ACTIVE);
}

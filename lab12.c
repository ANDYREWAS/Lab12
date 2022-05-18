/*
 * File:   lab12.c
 * Author: Josea
 *
 * Created on 18 de mayo de 2022, 11:18 AM
 */

/*
 * File:   Prelab12.c
 * Author: Josea
 *
 * Created on 16 de mayo de 2022, 11:22 AM
 */


// CONFIG1
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

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000

//
uint8_t flag = 0, potenciometro = 0, address = 0;

 //Prototipo de funciones 
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

 //INTERRUPCIONES 
 
void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // Fue interrupción del ADC?
        if(ADCON0bits.CHS == 0){    // Verificamos sea AN0 el canal seleccionado
            potenciometro = ADRESH;
            PORTC = ADRESH;         // Mostramos los bits superiores en PORTC
        }
        
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupción
    }
    
 
    if(INTCONbits.RBIF){                // Fue interrupci n del PORTB?
            
        if (!PORTBbits.RB1){
            flag = 0;
            PORTD = read_EEPROM(address);  
            INTCONbits.RBIF = 0;
        }
        if (PORTBbits.RB2 == 0){
            flag = 0;
            write_EEPROM(address, potenciometro);
            INTCONbits.RBIF = 0;
        }
        INTCONbits.RBIF = 0;
           
    return;
    }
}


 //CICLO PRINCIPAL

void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
        }
        if (!PORTBbits.RB0){
            flag = 1;
        }
        
        
        
        while (flag == 1){
        SLEEP();
        }
        
      
    }
    return;
}


 //CONFIGURACION 

void setup(void){
    ANSEL = 0b00000001; // AN0 como entrada analógica
    ANSELH = 0;         // I/O digitales)
    
    TRISA = 0b00000001; // AN0 como entrada
    PORTA = 0; 
    
    TRISC = 0;
    PORTC = 0;
    
    TRISD = 0;
    PORTD = 0;
    
    
    
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b0110;   // 4MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuraciones del ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    
    ADCON1bits.VCFG0 = 0;       // VDD *Referencias internas
    ADCON1bits.VCFG1 = 1;       // VSS
    
    ADCON0bits.CHS = 0b0000;    // Seleccionamos AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);
    
    // Configuracion de interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de int. ADC
    PIE1bits.ADIE = 1;          // Habilitamos int. de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
    //Pullups RB0 RB1
    TRISBbits.TRISB0 = 1;       // RB0 como entrada (configurada con bits de control)
    TRISBbits.TRISB1 = 1;       // RB1 como entrada (configurada con bits de control)
    OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB
    INTCONbits.RBIE = 1;
    WPUBbits.WPUB0 = 1;         // Habilitamos resistencia de pull-up de RB0
    WPUBbits.WPUB1 = 1;         // Habilitamos resistencia de pull-up de RB1
    IOCBbits.IOCB0 = 0;         // Habilitamos interrupcion por cambio de estado ?para RB0
    IOCBbits.IOCB1 = 1;         // Habilitamos interrupcion por cambio de estado ?para RB1
    IOCBbits.IOCB2 = 1;         // Habilitamos interrupcion por cambio de estado ?para RB2
    INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupcion

    
}



uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}
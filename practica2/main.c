// DSPIC33FJ32MC204 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = ON // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = EC // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128 // WDT Prescaler (1:128)
#pragma config WINDIS = OFF // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128 // POR Timer Value (128ms)
#pragma config ALTI2C = OFF // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1 // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#define baud_9600 51 // Para un valor de 2MHz

void delay_ms(unsigned long time_ms) {
    unsigned long u;
    for (u = 0; u < time_ms * 90; u++) // Cálculo aproximado para una CPU a 2MHz
    {
        asm("NOP");
    }
}

void EnviarCaracter(char c) {
    while (U1STAbits.UTXBF); // Mientras el buffer del puerto U1 este lleno, esperar en bucle while
    U1TXREG = c;             // Si no esta lleno, proceder a enviar el byte
}

void EnviarString(char * s) {
    while ((* s) != '\0') EnviarCaracter(* (s++)); // Mientras no se haya llegado al caracter nulo (final de trama), continuar imprimiendo datos.
}                                                  // *s es un puntero que apunta hacia la dirección del string de datos que le indiquemos. (*(s++) toma el contenido actual, y posteriormente aumenta el valor de la dirección (siguiente caracter))

void uart_config (unsigned int baud){    
    // Configuración de pines tx y rx
    TRISCbits.TRISC0  = 1;   // Pin de recepcion de uart establecido como entrada.
    RPINR18bits.U1RXR = 16;  // pin de recepcion rc0 trabajando con el modulo uart (RP16)
    RPOR8bits.RP17R   = 3;   // U1TX conectado con el pin RC1 (RP17)
    
    // Configuración de registro de U1MODE
    U1MODEbits.UARTEN = 0;     // Deshabilitar Uart.
    U1MODEbits.USIDL  = 0;     // Continuar operación en modo IDLE
    U1MODEbits.IREN   = 0;     // IR no usado
    U1MODEbits.RTSMD  = 1;     // Control de flujo desactivado.
    U1MODEbits.UEN    = 0;     // Solo usamos pin de Tx y pin de Rx
    U1MODEbits.WAKE   = 0;     // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;     // Loopback deshabilitado.
    U1MODEbits.ABAUD  = 0;     // Automedición de baudios (bps) deshabilidada
    U1MODEbits.URXINV = 0;     // En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH   = 1;     // Modo High-Speed
    U1MODEbits.PDSEL  = 0;     // 8 Bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL  = 0;     // 1-bit de stop al final de la trama de datos.   (8N1)

    // Configuración de registro de U1STA
    U1STAbits.UTXISEL0 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXISEL1 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXINV   = 0;    // El estado en reposo del pin de transmisión es High
    U1STAbits.UTXBRK   = 0;    // No usamos trama de sincronización
    U1STAbits.UTXEN    = 1;    // El transmisor a pleno funcionamiento.
    U1STAbits.URXISEL  = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.ADDEN    = 0;    // No usamos direccionamiento.
    //U1STAbits.RIDLE    = 0;
    U1STAbits.OERR     = 0;    // Reseteamos buffer de recepción

    // Configuramos la velocidad de transmisión/recepcción de los datos
    U1BRG = baud;

    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}

// Configuracion timer 1
void timer1_config(void) {
    T1CONbits.TON = 0;
    T1CONbits.TSIDL = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TSYNC = 0;
    T1CONbits.TCKPS = 2;

    PR1 = 0xFFFF;

    T1CONbits.TON = 1;
}

// Configuracion timer 3
void timer3_config(void) {
    T3CONbits.TON = 0;
    T3CONbits.TSIDL = 0;
    T3CONbits.TCS = 0;
    T3CONbits.TGATE = 0;
    // T3CONbits.TSYNC = 0; Unimplemented
    T3CONbits.TCKPS = 2;

    PR3 = 0xFFFF;

    T3CONbits.TON = 1;
}

char txbuffer[200];
unsigned int time_count = 0;
unsigned int contador_tmr3 = 0;
double tiempo_ms = 0;

int main(void) {
    //Configurar el oscilador para hacer funcionar la CPU a 2 MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    //Fosc = 8M * 2/(2 * 2) = 4 MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 4/2 = 2MHz (Frecuencia CPU)
    PLLFBD = 0; // M  = 2
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 0; // N2 = 2
    while (OSCCONbits.LOCK != 1); // Esperar a un PLL estable      

    AD1PCFGL = 0xFFFF; // Primer paso. Todos los pines configurados como pines digitales
   
    TRISAbits.TRISA0 = 0;
    TRISBbits.TRISB3 = 0;
    LATAbits.LATA0 = 0;

    TRISBbits.TRISB4 = 1; // Configurar el pin RB4 del puerto B como entrada  (P1) 
    TRISBbits.TRISB7 = 1; // Configurar el pin RB7 del puerto B como entrada  (P2) 

    uart_config(baud_9600);
    timer1_config();
    timer3_config();

    while (1) {
        TMR1 = 0;

        memset(txbuffer, '\0', sizeof (txbuffer)); // Limpiar el array txbuffer con un valor por defecto (NULL)

        // Apartado 2 
        if (time_count++ == 1) {
            if (!PORTBbits.RB4 && !PORTBbits.RB7) {
                sprintf(txbuffer, "%f P1: Pulsado P2: Pulsado \r\n", tiempo_ms);

            } else if (PORTBbits.RB4 && !PORTBbits.RB7) {
                sprintf(txbuffer, "%f P1: No Pulsado P2: Pulsado \r\n", tiempo_ms);

            } else if (!PORTBbits.RB4 && PORTBbits.RB7) {
                sprintf(txbuffer, "%f P1: Pulsado P2: No Pulsado \r\n", tiempo_ms);

            } else sprintf(txbuffer, "%f P1: No Pulsado P2: No Pulsado \r\n", tiempo_ms);

            time_count = 0; // Resetear variable para hacerlo cada 200ms
        }
        // Parpadeo led rojo
        LATBbits.LATB3 = !LATBbits.LATB3;

        // Apartado 3
        TMR3 = 0;
        EnviarString(txbuffer);
        contador_tmr3 = TMR3;
        tiempo_ms = (2.0 * ((double) contador_tmr3 + 1.0) / 2000000.0) * 1000.0;

 	// Bucle infinito apartado 1
        while (TMR1 < 3010);

    }
}
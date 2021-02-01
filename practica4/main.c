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

#define baud_9600 51// Para un valor de 2MHz

char txbuffer[200];
char dataCMD_ISR[50];

volatile int dutyOC1 = 1999;
volatile int dutyOC2 = 3999;

volatile unsigned int pulsoIC1 = 0;
volatile unsigned int pulsoIC2 = 0;

volatile unsigned int rise_pulsoIC1 = 0;
volatile unsigned int rise_pulsoIC2 = 0;

volatile unsigned int time_pulsoIC1 = 0;
volatile unsigned int time_pulsoIC2 = 0;

double realtime_pulsoIC1 = 0.00;
double realtime_pulsoIC2 = 0.00;

void delay_ms(unsigned long time_ms) {
    unsigned long u;
    for (u = 0; u < time_ms * 90; u++) // Cálculo aproximado para una CPU a 2MHz
    {
        asm("NOP");
    }
}

void EnviarCaracter(char c) {
    while (U1STAbits.UTXBF);
    U1TXREG = c;
}

void EnviarString(char * s) {
    while ((* s) != '\0') EnviarCaracter(* (s++));
}

void uart_config(unsigned int baud) {
    // Configuración de pines tx y rx
    TRISCbits.TRISC0 = 1; // Pin de recepcion de uart establecido como entrada.
    RPINR18bits.U1RXR = 16; // pin de recepcion rc0 trabajando con el modulo uart (RP16)
    RPOR8bits.RP17R = 3; // U1TX conectado con el pin RC1 (RP17)

    // Configuración de registro de U1MODE
    U1MODEbits.UARTEN = 0; // Deshabilitar Uart.
    U1MODEbits.USIDL = 0; // Continuar operación en modo IDLE
    U1MODEbits.IREN = 0; // IR no usado
    U1MODEbits.RTSMD = 1; // Control de flujo desactivado.
    U1MODEbits.UEN = 0; // Solo usamos pin de Tx y pin de Rx
    U1MODEbits.WAKE = 0; // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0; // Loopback deshabilitado.
    U1MODEbits.ABAUD = 0; // Automedición de baudios (bps) deshabilidada
    U1MODEbits.URXINV = 0; // En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH = 1; // Modo High-Speed
    U1MODEbits.PDSEL = 0; // 8 Bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL = 0; // 1-bit de stop al final de la trama de datos.   (8N1)

    // Configuración de registro de U1STA
    U1STAbits.UTXISEL0 = 0; // Tema interrupciones (no mirar aun)
    U1STAbits.UTXISEL1 = 0; // Tema interrupciones (no mirar aun)
    U1STAbits.UTXINV = 0; // El estado en reposo del pin de transmisión es High
    U1STAbits.UTXBRK = 0; // No usamos trama de sincronización
    U1STAbits.UTXEN = 1; // El transmisor a pleno funcionamiento.
    U1STAbits.URXISEL = 0; // Tema interrupciones (no mirar aun)
    U1STAbits.ADDEN = 0; // No usamos direccionamiento.
    U1STAbits.RIDLE = 0;
    U1STAbits.OERR = 0; // Reseteamos buffer de recepción

    // Configuramos la velocidad de transmisión/recepcción de los datos
    U1BRG = baud;

    // Prioridades, flags e interrupciones correspondientes a la Uart
    IPC2bits.U1RXIP = 6; // U1RX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1RXIF = 0; // Reset Rx Interrupt flag
    IEC0bits.U1RXIE = 1; // Enable Rx interrupts

    U1MODEbits.UARTEN = 1; // Uart habilitada por completo
}

// Configuracion timer 2
void timer2_config(void) {
    T2CONbits.TON = 0;
    T2CONbits.TSIDL = 0;
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = 0;
    T2CONbits.T32 = 0;
    T2CONbits.TCS = 0;

    PR2 = 10000;

    T2CONbits.TON = 1;
}

// Configuracion timer 3
void timer3_config(void) {
    T3CONbits.TON = 0;
    T3CONbits.TSIDL = 0;
    T3CONbits.TGATE = 0;
    T3CONbits.TCKPS = 0;
    T3CONbits.TCS = 0;

    T3CONbits.TON = 1;
}

// Configuracion modulos OC
void OC_config(void) {
    // Pines remapeables usados para salida modulo PWM
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC3 = 0;
    RPOR9bits.RP18R = 0x12; // RC2 
    RPOR9bits.RP19R = 0x13; // RC3 

    // Iniciacion Ouput Compare 1
    OC1CONbits.OCM = 0;
    OC1CONbits.OCSIDL = 0;
    OC1CONbits.OCFLT = 0;
    OC1CONbits.OCTSEL = 0; // Trabaja con el timer 2

    OC1RS = 2000; // Ciclo de trabajo a 1ms
    // Ciclo de trabajo OC1: (Ton/T)*100 = (OC1RS/PR2)*100 = (1/5)*100 = 20% 

    // Iniciacion Ouput Compare 2
    OC2CONbits.OCM = 0;
    OC2CONbits.OCSIDL = 0;
    OC2CONbits.OCFLT = 0;
    OC2CONbits.OCTSEL = 0;

    OC2RS = 4000; // Ciclo de trabajo a 2ms
    // Ciclo de trabajo OC2: (Ton/T)*100 = (OC1RS/PR2)*100 = (2/5)*100 = 40% 

    // Activacion ambos modulos
    OC1CONbits.OCM = 6; // Activar OC1 pin RC2
    OC2CONbits.OCM = 6; // Activar OC2 pin RC3
}

// Configuracion modulos IC
void IC_config(void) {
    // Pines remapeables usados para entrada modulo IC
    TRISCbits.TRISC5 = 1;
    TRISCbits.TRISC6 = 1;
    RPINR7bits.IC1R = 0x15; // RC5 
    RPINR7bits.IC2R = 0x16; // RC6 

    // Iniciacion Input Capture 1
    IC1CONbits.ICM = 0;
    IC1CONbits.ICSIDL = 0;
    IC1CONbits.ICTMR = 0; // Select Timer3 as the IC1 Time base
    IC1CONbits.ICI = 0; // Interrupt on every second capture event

    // Iniciacion Input Capture 2
    IC2CONbits.ICM = 0;
    IC2CONbits.ICSIDL = 0;
    IC2CONbits.ICTMR = 0; // Select Timer3 as the IC2 Time base
    IC2CONbits.ICI = 0; // Interrupt on every second capture event

    // Prioridades, flags e interrupciones 
    IPC0bits.IC1IP = 5; // Setup IC1 interrupt priority level
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1; // Enable IC1 interrupt
    IPC1bits.IC2IP = 5; // Setup IC2 interrupt priority level
    IFS0bits.IC2IF = 0; // Clear IC2 Interrupt Status Flag
    IEC0bits.IC2IE = 1; // Enable IC2 interrupt

    // Activacion ambos modulos
    IC1CONbits.ICM = 3; // Generate capture event on every Rising edge
    IC2CONbits.ICM = 3; // Generate capture event on every Rising edge
}

// Vector de interrupcion UART_RX ISR
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    dataCMD_ISR[0] = U1RXREG; // Obtener caracter recibido en el buffer

    if (dataCMD_ISR[0] == '+') dutyOC1 = dutyOC1 + 100;
    if (dataCMD_ISR[0] == '-') dutyOC1 = dutyOC1 - 100;

    if(dutyOC1 < 0|| dutyOC1 > 10000) dutyOC1 = 1999;
    
    OC1RS = dutyOC1;

    if (dataCMD_ISR[0] == '1') dutyOC2 = dutyOC2 + 100;
    if (dataCMD_ISR[0] == '2') dutyOC2 = dutyOC2 - 100;

    if(dutyOC2 < 0|| dutyOC2 > 10000) dutyOC2 = 3999;
    
    OC2RS = dutyOC2;

    IFS0bits.U1RXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void) {
    if (pulsoIC1 == 0) {
        rise_pulsoIC1 = IC1BUF; // Capture next falling edge
        IC1CONbits.ICM = 2;
        pulsoIC1 = 1;
    } else {
        time_pulsoIC1 = IC1BUF - rise_pulsoIC1;
        IC1CONbits.ICM = 3; // Capture next rising edge
        pulsoIC1 = 0;
    }
    IFS0bits.IC1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void) {
    if (pulsoIC2 == 0) {
        rise_pulsoIC2 = IC2BUF; // Capture next falling edge
        IC2CONbits.ICM = 2;
        pulsoIC2 = 1;
    } else {
        time_pulsoIC2 = IC2BUF - rise_pulsoIC2;
        IC2CONbits.ICM = 3; // Capture next rising edge
        pulsoIC2 = 0;
    }
    IFS0bits.IC2IF = 0;
}

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

    // Configurar UART, timers, modulo PWM e Input Capture
    uart_config(baud_9600);
    timer2_config();
    timer3_config();
    OC_config();
    IC_config();
    delay_ms(10); // Dejamos unos ms para cargar y que no imprima tiempo 0

    // Habilitar interrupciones globales
    INTCON1bits.NSTDIS = 0; // Interrupt nesting enables
    SRbits.IPL = 0; // Enable global interrupts

    while (1) {
        if (U1STAbits.OERR) U1STAbits.OERR = 0; // Si hay overflow en el buffer de recepcion resetea uart
   
        // Imprimir Duty Cycle cada 200ms
        realtime_pulsoIC1 = 1.0 * ((double) time_pulsoIC1) / 2000.0;
        realtime_pulsoIC2 = 1.0 * ((double) time_pulsoIC2) / 2000.0;

        sprintf(txbuffer, "Tiempo IC1: %.3lfms - Tiempo IC2: %.3lfms\r\n", realtime_pulsoIC1, realtime_pulsoIC2);
        EnviarString(txbuffer);

        delay_ms(200);
    }
}
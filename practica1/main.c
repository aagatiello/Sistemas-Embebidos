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
#include <string.h>
#include <stdbool.h>

#define baud_9600 1041
#define led_D1 LATBbits.LATB3
#define led_D2 LATAbits.LATA0

void delay_ms(unsigned long time_ms) {
    unsigned long u;
    for (u = 0; u < time_ms * 450; u++) // Cálculo aproximado para una CPU a 4MHz
    {
        asm("NOP");
    }
}

void uart_config(unsigned int baud) {
    // Configuracion de pines Tx y Rx
    TRISCbits.TRISC0  = 1;    // Pin de recepcion de uart establecido como entrada
    RPINR18bits.U1RXR = 21;   // Pin de recepcion RC5 trabajando con el modulo uart 
    RPOR9bits.RP19R   = 3;    // U1TX conectado con el pin RC3
    
    // Configuracion de resgistro de U1MODE
    U1MODEbits.UARTEN = 0;    // Deshabilitar la uart.
    U1MODEbits.USIDL  = 0;    // Continuar operacion en modo idle
    U1MODEbits.IREN   = 0;    // IR no usado
    U1MODEbits.RTSMD  = 1;    // Control de flujo desactivado
    U1MODEbits.UEN    = 0;    // Solo usamos pin de Tx y pon de Rx
    U1MODEbits.WAKE   = 0;    // No quiero que la uart despierte del modo sleep
    U1MODEbits.LPBACK = 0;    // Loopback deshabilitado
    U1MODEbits.ABAUD  = 0;    // Automedicion de baudios (bps) deshabilitada
    U1MODEbits.URXINV = 0;    // En estado de reposo, el receptor mantiene un estado alto (high)
    U1MODEbits.BRGH   = 1;    // Modo High-Speed
    U1MODEbits.PDSEL  = 0;    // 8 bits de datos y paridad nula (8N)
    U1MODEbits.STSEL  = 0;    // 1-bit de stop al final de la trama de datos (8N1)   
    
    // Configuracion de registro U1STA
    U1STAbits.UTXISEL0 = 0;   // Tema interrupciones (no mirar aun)
    U1STAbits.UTXISEL1 = 0;   // Tema interrupciones (no mirar aun)
    U1STAbits.UTXINV   = 0;   // El estado de reposo del pin de transimision es High
    U1STAbits.UTXBRK   = 0;   // No usamos trama de sincronizacion
    U1STAbits.UTXEN    = 1;   // Transmisor a pleno funcionamiento
    U1STAbits.URXISEL  = 0;   // Tema interrupciones (no mirar aun)
    U1STAbits.ADDEN    = 0;   // No usamos direccionamiento
    // U1STAbits.RIDLE = 0; 
    U1STAbits.OERR     = 0;   // Resetea buffer de recepcion
    
    // Configuracion velocidad de transmision/recepcion datos
    U1BRG = baud;
    
    U1MODEbits.UARTEN = 1;    // Uart habilitada por completo
}

void uart_send_byte(unsigned char var){
    while(U1STAbits.TRMT == 0);
    U1TXREG = var;
}

int main(void) {
    //Configurar el oscilador para funcionar a 40 MHz
    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    //Fosc = 8M * 40/(2 * 2) = 80 MHz para un reloj de 8MHz de entrada
    PLLFBD = 38; // M = 40
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 0; // N2 = 2
    while (OSCCONbits.LOCK != 1); // Wait for PLL to lock
    
    AD1PCFGL = 0xFFFF; 
    
    TRISBbits.TRISB3 = 0; // Configurar el pin RB3 del puerto B como salida (D1)
    TRISAbits.TRISA0 = 0; // Configurar el pin RA0 del puerto A como salida (D2)
    
    delay_ms(10); 
    
    LATBbits.LATB3 = 0; // Escribir un '0' o estado low, por defecto, en el pin RB3 (D1)
    LATAbits.LATA0 = 0; // Escribir un '0' o estado low, por defecto, en el pin RA0 (D2)
    
    delay_ms(10); 
    
    uart_config(baud_9600);
    
    unsigned char var = 0;
    unsigned int contador = 0;
    char buffer[80];
    bool check = false;
    
    while (1) {
        // Apartado 1
        sprintf(buffer, "Agustina %d %c \n", contador);
        puts(buffer);
        contador = contador++;
        
        if (U1STAbits.URXDA == 1) {
            var = U1RXREG;
            uart_send_byte(var);
            
            // Apartado 2
            if (var == 101 || var == 69) led_D1 = 1;       // Si e ó E (ASCII) encender led rojo (D1)
            else if (var == 97 || var == 65) led_D1 = 0;   // Si a ó A (ASCII) apagar led rojo (D1)
            
            // Apartado 3 - principio
            if (var == 104 || var == 72) {                 // Si h ó H (ASCII)
                check = !check;   
            }
            
            // Apartado 4
            if (var == 32) contador = 0;                   // Si barra espaciadora
        }
        
        // Apartado 3 - continuacion
        if (check) {
            led_D2 = !led_D2;
            delay_ms(250);
        } else led_D2 = 0;
        
        U1TXREG = '\r';
        delay_ms(500);
    }
    
    return 0;
}
#include <xc.h>
#include "nxlcd.h"
// Configura??es
#pragma config PLLDIV = 5 // PLL para 20MHz
#pragma config CPUDIV = OSC1_PLL2 // PLL desligado
#pragma config FOSC = HS // Fosc = 20MHz; Tcy = 200ns
#pragma config WDT = OFF // Watchdog timer desativado
#pragma config PBADEN = OFF // Pinos do PORTB come?am como digitais
#pragma config LVP = OFF // Desabilita grava??o em baixa tens?o
#pragma config DEBUG = ON // Habilita debug
#pragma config MCLRE = ON // Habilita MCLR e desabilita RE3 como I/O
#define _XTAL_FREQ 20000000 

#pragma config	CCP2MX = ON // Pino RC1 utilizado em CCP2

unsigned int Kp = 5, temperaturaReferencia = 35, controleProporcional = 0, estouros = 0, valorLido1 = 0, valorLido2 = 0;
unsigned int razaoCiclicaCCP1 = 100;
float temperaturaAtual = 0, voltage = 0;
unsigned int aumenta = 0, diminui = 0, funcdisp = 0, conversor_temp = 0, attRazaoCiclicaCCP1 = 0, attRazaoCiclicaCCP2 = 0, controlador = 0, potencia = 0;


void interrupt HighPriorityISR(void)
{
    if(PIR1bits.ADIF == 1){
        PIR1bits.ADIF = 0;
        if(estouros == 50){//estoura na freq de 30Hz 
            estouros = 0;
            potencia = 1;
        }else{//estoura na freq de 1500Hz            
            estouros++;
            conversor_temp = 1;
        }        
    }
}
void interrupt low_priority LowPriorityISR(void)
{   
    if(INTCONbits.TMR0IF){//estoura na frequencia de 1500hz -> 0.00066666s
        INTCONbits.TMR0IF = 0;
        if(estouros == 50){                        
            ADCON0bits.CHS1 = 1;
            ADCON0bits.CHS0 = 1;
        }else{
            ADCON0bits.CHS1 = 0;
            ADCON0bits.CHS0 = 0;            
        }
        controlador = 1;        
        ADCON0bits.GODONE = 1;
        TMR0L = 0x30;
    }
    if(INTCON3bits.INT1IF == 1){
        INTCON3bits.INT1IF = 0;
        aumenta = 1;
    }
    if(INTCON3bits.INT2IF == 1){
        INTCON3bits.INT2IF = 0;
        diminui = 1;        
    }
}


void main(void)
{   
    //entrada e saida do potenciometro
    TRISAbits.RA3 = 1;//entrada
    TRISCbits.RC1 = 0;//saida
    
    
    //saida do ventilador controlado pelo PWM e CCP1
    TRISCbits.RC2 = 0;    
    
    //entrada dos botoes INT1 e INT2
    TRISBbits.RB0 = 1;
    TRISBbits.RB1 = 1; 
    
    
    //Configuracao das interrupcoes 
    RCONbits.IPEN = 1;//NIVEIS DE PRIORIDADE LIGADO
    INTCONbits.GIEH = 1;//ALTA PRIORIDADE LIGADO
    INTCONbits.GIEL = 1;//BAIXA PRIORIDADE LIGADO
    
    
    
    
    //definindo portas anal?gicas    
    ADCON1bits.PCFG3 = 1;
    ADCON1bits.PCFG2 = 0;
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG0 = 1;//AN3, AN2, AN1 e AN0 anal?gicas
    
    ADCON0bits.ADON = 1;//Conversos A/D ligado
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    
    ADCON0bits.CHS3 = 0;
    ADCON0bits.CHS2 = 0;
    
    ADCON2bits.ADFM = 1;// DIREITA
    ADCON2bits.ACQT2 = 0;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;// 4 TAD
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;// FOSC/16
        
    PIE1bits.ADIE = 1;//interrupcao do sinal A/D ligada
    PIR1bits.ADIF = 0;//flag zerada
    IPR1bits.ADIP = 1;//prioridade baixa da interrupcao 
    
    
    //Configura??o do Timer0
    T0CONbits.TMR0ON = 1; //timer0 ligado      
    T0CONbits.T08BIT = 1; //timer0 8 bits
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 0;
    T0CONbits.T0PS2 = 0;//
    T0CONbits.T0PS1 = 1;//
    T0CONbits.T0PS0 = 1;//pre-scale de 16
    
    INTCONbits.TMR0IE = 1;//interrup??o ativada
    INTCONbits.TMR0IF = 0;//flag zerada
    INTCON2bits.TMR0IP = 0;//prioridade baixa     
    
    //1/1500 = 0.00066666
    //0.00066666 = 0,2*10^-6*16*(256 - [48] )
    //48 = 0x30
    //TMR0H = 0x00;
    TMR0L = 0x30;
    
    
    //pull-up de bot?es
    INTCON2bits.RBPU = 0;
        
    //Configura??o do INT1
    INTCON3bits.INT1E = 1;//prioridade ligada
    INTCON3bits.INT1IF = 0;//flag zerada    
    INTCON3bits.INT1IP = 0;//baixa prioridade
    INTCON2bits.INTEDG1 = 0;//borda de descida
    
    //Configura??o do INT2
    INTCON3bits.INT2IE = 1;//prioridade ligada
    INTCON3bits.INT2IF = 0;//flag zerada    
    INTCON3bits.INT2IP = 0;//baixa prioridade
    INTCON2bits.INTEDG2 = 0;//borda de descida
    
    
    
    //configurar o PWM do ventilador gerado pelo CCP1
    //T = 1/6500 = 0.0001538
    T2CONbits.TMR2ON = 1; //TIMER2 LIGADO
    T2CONbits.T2CKPS1 = 1; //PRE-SCALE do TMR2 DE 16    
    //PR2 = (0.0001538/(0.2*10^-6*16))-1
    PR2 = 47;
    CCP2CONbits.CCP2M3 = 1;
    CCP2CONbits.CCP2M2 = 1;//CCP2 modo PWM    
    

    CCP1CONbits.CCP1M3 = 1;
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M0 = 0;//CCP em pwm 
    
    attRazaoCiclicaCCP1 = 1;
    attRazaoCiclicaCCP2 = 1;
    
    OpenXLCD(FOUR_BIT & LINES_5X7);
    WriteCmdXLCD(0x01);
    WriteCmdXLCD(0x0C);
    __delay_ms(2);
    
    while(1){
        if(aumenta){//aumenta o valor de referencia do displau
            aumenta = 0;
            if(temperaturaReferencia<50){  
                temperaturaReferencia++;
            }else{
                temperaturaReferencia = 50;
            }
        }
        if(diminui){//diminui o valor de referencia do display
            diminui = 0;
            if(temperaturaReferencia>35){
                temperaturaReferencia--;
            }else{
                temperaturaReferencia = 35;
            }
        }
        if(conversor_temp){//captura o sensor de temperatura -> atualiza o VALOR numa frequencia de 1500HZ
            conversor_temp = 0;
            valorLido1 = 256 * ADRESH + ADRESL;
            temperaturaAtual = valorLido1 * 0.4887585533;
        }
        if(potencia){//captura o potenciometro -> Atualiza o VALOR numa frequencia de 30HZ
            potencia = 0;
            valorLido2 = 256 * ADRESH + ADRESL;
            voltage = 70 + (valorLido2 * 0.0293255132);
            attRazaoCiclicaCCP2 = 1;
            funcdisp = 1;//Atualiza o display
        } 
        if(attRazaoCiclicaCCP2){//Gera o sinal do PWM do ventilador a partir do CCP1
            attRazaoCiclicaCCP2 = 0;
            int dutyCicle = 4 * 48 * voltage / 100;
            CCPR2L = (char)(dutyCicle >> 2);
            CCP2CONbits.DC2B1 = (dutyCicle >> 1) % 2;
            CCP2CONbits.DC2B0 = dutyCicle % 2;
        }
        
        if(attRazaoCiclicaCCP1){//Gera o sinal do PWM do ventilador a partir do CCP1
            attRazaoCiclicaCCP1 = 0;
            int dutyCicle = 4 * 48 * razaoCiclicaCCP1 / 100;
            CCPR1L = (char)(dutyCicle >> 2);
            CCP1CONbits.DC1B1 = (dutyCicle >> 1) % 2;
            CCP1CONbits.DC1B0 = dutyCicle % 2;
        }
        if(controlador){//aumenta o duty cycle
            controlador = 0;
            
            int erro = temperaturaAtual - temperaturaReferencia;

            controleProporcional = Kp * erro;
            
            if(erro > 0){                
                razaoCiclicaCCP1 = controleProporcional + 20;
            }
            if(erro <= 0){
                razaoCiclicaCCP1 = 20;
            }                    

            attRazaoCiclicaCCP1 = 1;
        }        
        if(funcdisp){//atualiza o display na frequencia de 30 Hz -> 0,0333333s
            funcdisp = 0;   
            WriteCmdXLCD(0x80);
            putrsXLCD("Tref:"); 
            putcXLCD(0x30 + (temperaturaReferencia/10));        
            putcXLCD(0x30 + (temperaturaReferencia%10));
            WriteCmdXLCD(0xC0);
            
            WriteCmdXLCD(0x88);
            putrsXLCD("PW:");
            putcXLCD (0x30 + razaoCiclicaCCP1/10);
            putcXLCD (0x30 + razaoCiclicaCCP1%10);
            putrsXLCD ("%");

            WriteCmdXLCD(0xC0); 
            putrsXLCD ("Temp:");
            putcXLCD (0x30 + ((int)(temperaturaAtual))/10);
            putcXLCD (0x30 + ((int)(temperaturaAtual))%10);
            putrsXLCD ("C");
        }
    }
}

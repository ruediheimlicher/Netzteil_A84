//
//  Netzteil_ATTiny84.c
//
//  Copyright Ruedi Heimlicher 2021. All rights reserved.
//


#include "lcd.c"

#include "adc.c"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "defines.h"

//***********************************
//									
//***********************************



//#define OUTPORT	PORTD		// Ausgang fuer Motor

//#define INPORT   PORTD  // Input signal auf INT0
//#define INPIN   PIND  // Input signal

#define DATAPIN  2 





#define BEEP_ONTIME  16
#define BEEP_OFFTIME  64
#define BEEP_OFFTIMEDELAY  196


volatile    uint16_t timercount0=0;
volatile    uint16_t timercount1=0;
volatile    uint8_t beepcounter=0;
volatile    uint8_t beeptime=4; // 
volatile    uint8_t beepburstcounter=0; // 

volatile    uint8_t beep_ontime=2;
volatile    uint8_t beep_offtime=6;

volatile    uint8_t adccount0=0;
volatile    uint8_t blinkcount=0;
volatile  uint8_t pwmimpuls = 0;

volatile    uint8_t pwmpos=0;

volatile    uint16_t led_temp=0; // Eingang von Kuehlkoerper, sinkend
volatile    uint16_t stromreg = 0; // Eingang von Stromregelung, sinkend

volatile    uint8_t status=0;

uint8_t loopledtakt = 0x0F;

void slaveinit(void)
{
 	OSZIPORT |= (1<<OSZIA);	//Ausgang fuer OSZI A
	OSZIDDR |= (1<<OSZIA);	//Ausgang fuer OSZI A

   LOOPLEDDDR |=(1<<LOOPLED); // HI
   LOOPLEDPORT |=(1<<LOOPLED);

   OUTDDRA |= (1<<BLINK_PIN);      //Pin 1 von PORT D als Ausgang fuer Schalter: OFF
   OUTPORTA &= ~(1<<BLINK_PIN); // LO
   
   OUTDDRB |= (1<<BEEP_PIN);      //Pin 2 von PORT D als Ausgang fuer Buzzer
   
   OUTDDRB |= (1<<PWM_FAN_PIN);      //Pin 3 von PORT D als Ausgang fuer LED TWI
   OUTPORTB &= ~(1<<PWM_FAN_PIN); // LO   
   
   OUTDDRA |= (1<<OUT_OFF_PIN); // Aushang fuer Not-OFF
   OUTPORTA &= ~(1<<OUT_OFF_PIN); // LO

 

}


void timer0 (void) 
{ 
// Timer fuer Exp
//   TCCR0 |= (1<<CS00)|(1<<CS02);   //Takt /1024
//   TCCR0 |= (1<<CS02);            //8-Bit Timer, Timer clock = system clock/256
   TCCR0A = 0;
   TCCR0A |= (1<<WGM01);
//Timer fuer Servo   
   TCCR0B |= (1<<CS01);   //Takt /64 Intervall 64 us
   
   //TIFR |= (1<<TOV0);             //Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
//   TIMSK0 |= (1<<TOIE0);         //Overflow Interrupt aktivieren
   TIMSK0 |= (1<<OCIE0A);
   TCNT0 = 0x00;               //RŸcksetzen des Timers
   OCR0A = 100;
}


ISR(TIM0_COMPA_vect)
{
   //OSZITOGG;
   //OUTPORT &= ~(1<<BEEP_PIN);
   if (status & (1<<BEEP_ON))
   {
      //OSZITOGG;
      OUTPORTB ^= (1<<BEEP_PIN);
   }
}

ISR(TIM0_OVF_vect)
{
  // OSZIHI;
   {
 //     OUTPORT ^= (1<<BLINK_PIN); // 
   }
   
}
//MARK:  timer 1
// Timer1 fuer Takt der Messung
void timer1(void)
{
   // https://andreasrohner.at/posts/Electronics/How-to-set-the-PWM-frequency-for-the-Attiny84/
   TCCR1B = 0;

//   TCCR2A |= (1<<WGM21);// Toggle OC2A
   TCCR1A |=  (1<<WGM11) ; //| (1<<WGM10);
 
     TCCR1B |= (1<<WGM12) | (1<<WGM13);
 //  TCCR2A |= (1<<WGM20);
 //  TCCR2A |= (1<<COM2A0);                  // CTC
   
   /*
    CS22   CS21   CS20   Description
    0    0     0     No clock source
    0    0     1     clk/1
    0    1     0     clk/8
    0    1     1     clk/32
    1    0     0     clk/64
    1    0     1     clk/128
    1    1     0     clk/256
    1    1     1     clk/1024
    */
   
   //TCCR2B |= (1<<CS22); //
   //TCCR2B |= (1<<CS21);//
  // TCCR1B |=  (1<<CS11)   ;
   TCCR1B |= (1<<CS11) | (1<<CS11);
   
   TIMSK1 |= (1<<OCIE1A);      //  Interrupt A En
   
   TIMSK1 |=(1<<TOIE1);                  //Overflow Interrupt aktivieren
   TCNT1 = 0;                             //RŸcksetzen des Timers
   //OSZILO;
   //ICR1 = 0xFF;
   ICR1 = TIM1_TOP;
   OCR1A = TIM1_TOP-2; // 20ms
 //  OCR1B =100;
//   OCR2A = 0x02;
   //ICR1 = 0xFF;
   //DDRB |= (1<<PORTB3);
//   TIFR1 |= (1<<TOV1);                     //Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
}

//MARK: TIM1_compartor
// Timer2 fuer Takt der Messung und Signal an Triac
ISR(TIM1_COMPA_vect) // CTC Timer2
{
   //
   //OSZITOGG;
   //return;
   timercount0++;
   OSZILO;
   if (status & (1<<FAN_ON))
   {
      OUTPORTB |= (1<<PWM_FAN_PIN); // Fan ON
   }
   
   if (timercount0 > 5) // Takt teilen, 1s
   {
       //OSZITOGG;
      timercount0=0;
      
      if ((status & (1<<FAN_ON)) )
      {
         //OSZITOGG;
         if (beepburstcounter > 2)
         {
            beep_offtime = BEEP_OFFTIMEDELAY;
         }
         
         if (beepcounter == beep_offtime)
         {
           //OSZITOGG;
            OUTPORTA |= (1<<BLINK_PIN);
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            OUTPORTA &= ~(1<<BLINK_PIN);
            status &= ~(1<<BEEP_ON);
         }
         beepcounter++;
      }
      else if ( (status & (1<<STROM_ON)))
      {
        
         if (beepburstcounter > 2)
         {
            beep_offtime = BEEP_OFFTIMEDELAY;
         }
         
         if (beepcounter == beep_offtime)
         {
            //OSZITOGG;
            OUTPORTA |= (1<<BLINK_PIN);
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            OUTPORTA &= ~(1<<BLINK_PIN);
            status &= ~(1<<BEEP_ON);
            
         }
         beepcounter++;
      }
      else
      {
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
            beep_offtime = BEEP_OFFTIME;
         }
         beepcounter++;
      }
 
      timercount1++;
      
      status |= (1<<PWM_ADC);// ADC messen ausloesen
      
        
   }
}
/*
ISR(TIM1_COMPB_vect) // CTC Timer2
{
   //OSZITOGG;
}
*/
ISR(TIM1_OVF_vect) // CTC Timer2
{
   OSZIHI;
   OUTPORTB &= ~(1<<PWM_FAN_PIN); // Fan OFF
  //OSZITOGG;
}

//MARK: main

void main (void) 
{
   //WDT ausschalten 
   MCUSR = 0;
   wdt_disable();
   slaveinit();
   // int0_init();
   
   timer0();
   timer1();
   initADC(0);
   
   
   uint8_t loopcount0=0;
   uint8_t loopcount1=0;
   //  lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(50);
   
   
   
   // WDT
   // https://bigdanzblog.wordpress.com/2015/07/20/resetting-rebooting-attiny85-with-watchdog-timer-wdt/
   /*
    WDTCSR|=(1<<WDCE)|(1<<WDE);  // https://www.instructables.com/ATtiny85-Watchdog-reboot-Together-With-SLEEP-Andor/
    WDTCSR=0x00; // disable watchdog
    */
    //  lcd_gotoxy(0,0);
   //  lcd_puts("Netzteil_84");
   sei();
   while (1)
   {	
      //OSZITOGG;
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      
      // Strom
      
      stromreg = readKanal(ADC_STROM_PIN); 
      
      if (stromreg < STROM_MIN)
      {
         //OSZITOGG;
         //OCR2A = TIMER2_COMPA_STROM;
         OCR0A = OCR0A_STROM;
         if (!(status & (1<<STROM_ON)))
         {
            status |= (1<<STROM_ON);
            beepcounter = BEEP_OFFTIME-1;
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
         }
      }
      else if (stromreg > STROM_MIN +1)
      {
         if ((status & (1<<STROM_ON)))
         {
            status &= ~(1<<STROM_ON);
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
            status &= ~(1<<BEEP_ON);
         }
      }
      
      
      // end Strom
      
      //MARK: ADC
      if (status & (1<<PWM_ADC)) // ADC tempsensor lesen, beep einschalten 
      {
         
         status &= ~(1<<PWM_ADC);
         
         led_temp = readKanal(ADC_TEMP_PIN); 
         
         pwmimpuls = 4*(led_temp-TEMP_OFFSET); // groesserer Bereich
         
         
         if (pwmimpuls > (TIM1_TOP-2))
         {
            pwmimpuls = TIM1_TOP-2;
            status &= ~(1<<FAN_ON); // Fan OFF 
            //  OCR1A = pwmimpuls;
            OUTPORTB &= ~(1<<PWM_FAN_PIN); // Fan OFF
         }
         //if (status & (1<<FAN_ON))
         {
            OCR1A = pwmimpuls;
         }
    //     OCR1A = TIM1_TOP -70 + (led_temp-TEMP_OFFSET) ;
         
      }
      
      
      //MARK: Temp      
      // Temperatur
      if (led_temp < TEMP_FAN)
      {
         //OCR2A = TIMER2_COMPA_TEMP;
         OCR0A = OCR0A_TEMP;
         if (!(status & (1<<FAN_ON)))
         {
            status |= (1<<FAN_ON);
            beepcounter = BEEP_OFFTIME-1;
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
         }
         
         if (led_temp < TEMP_OFF)
         {
            OUTPORTA |= (1<<OUT_OFF_PIN); // output OFF
         }
      }
      else if (led_temp > (TEMP_FAN + 1))
      {
         if ((status & (1<<FAN_ON)))
         {
            status &= ~(1<<FAN_ON);
            OUTPORTA &= ~(1<<OUT_OFF_PIN); // Output wieder ON
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
            status &= ~(1<<BEEP_ON);
         }
      }
      
      
      
      //LOOPLEDPORT ^= (1<<LOOPLED);
      
      
      // Blinky
      if (loopcount0>=loopledtakt)
      {
         //OSZITOGG;
         loopcount0=0;
         loopcount1++;
         if (loopcount1 >= loopledtakt)
         {
            LOOPLEDPORT ^= (1<<LOOPLED); // Kontrolle lastDIR
            loopcount1 = 0;
            
         }
         
      }
      
      //OSZIAHI;
   }//while
}

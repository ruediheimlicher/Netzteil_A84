//
//  defines.h
//  H0_Decoder
//
//  Created by Ruedi Heimlicher on 11.09.2020.
//

#ifndef defines_h
#define defines_h

#define LOOPLEDPORT     PORTB
#define LOOPLEDDDR      DDRB
#define LOOPLED         0 // 

// ADC
#define ADCPORT PORTA
#define ADCDDR    DDRA
#define ADC_TEMP_PIN    0 // von Kuehlkoerper
#define ADC_STROM_PIN   1 // von OP Amp, sinkend

#define OUTPORTA   PORTA
#define OUTDDRA    DDRA

#define OUT_OFF_PIN     2 // Not-Aus, active HI
#define BLINK_PIN       3 // OUT active HI

#define OUTPORTB   PORTB
#define OUTDDRB    DDRB

#define BEEP_PIN        1 // OUT altern
#define PWM_FAN_PIN     2 // OUT active HI


// Bits status
#define PWM_ON       0
#define PWM_ADC      1
#define FAN_ON       3

#define BEEP_ON      4

#define STROM_ON     5 // Stromregelung wirkt

#define STROM_MIN    300

#define TEMP_MAX     600 // Fan starten, beep
#define TEMP_OFF     560 // Ausschalten
#define TEMP_DELAY   550



#define TIMER1_BLINK_TAKT 2

#define TIMER1_COMPA 0x2FF // OV 20ms

#define MAXTEMP 128

#define OCR0A_TEMP 0x80;
#define OCR0A_STROM 0xA0;

#define OSZIPORT  PORTA      // 
#define OSZIDDR   DDRA

#define OSZIA 7           // 

#define OSZILO OSZIPORT &= ~(1<<OSZIA)
#define OSZIHI OSZIPORT |= (1<<OSZIA)
#define OSZITOGG OSZIPORT ^= (1<<OSZIA)




#endif /* defines_h */

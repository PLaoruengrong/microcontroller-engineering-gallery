#include <avr/io.h>
#include <avr/interrupt.h>
#define SERVO_MIN 1000
#define SERVO_MAX 5000

volatile uint8_t blink_enabled = 0;
volatile uint8_t ovf_count = 0;

ISR(TIMER2_OVF_vect){
  static int8_t dir = 1;
  static uint8_t duty = 0;
  duty += dir;
  if(duty == 0 || duty == 255) dir = -dir;
  OCR2B = duty;
}

ISR(ADC_vect){
  uint16_t adc = ADC; // 0..1023
  OCR1A = SERVO_MIN + ((uint32_t)adc * (SERVO_MAX - SERVO_MIN)) /1023UL;
  blink_enabled = (adc >= 512);
  ADCSRA |= (1 << ADSC);
}

ISR(TIMER0_OVF_vect){
  if(blink_enabled){
    if(++ovf_count >= 31){
      PINB = (1 << PB5); // Toggle D13
      ovf_count = 0;
    }
  }
  else{
    PORTB &= ~(1 << PB5); // Keep D13 off
  }
}

int main(void){
  // ADC on PC0/A0
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  DIDR0 = (1 << ADC0D);

  // Servo on D9 (PB1/OC1A)
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;

  // Blink LED on D13 via Timer0
  DDRB |= (1 << PB5);
  TCCR0A = 0;
  TCCR0B = (1 << CS02) | (1 << CS00);
  TIMSK0 = (1 << TOIE0);

  // Continuous breathe on D3 via Timer2
  DDRD |= (1 << PD3);
  TCCR2A = (1 << COM2B1) | (1 << WGM200);
  TCCR2B = (1 << CS22) | (1 << CS21);
  TIMSK2 = (1 << TOIE2);

  sei();
  while(1){}
}

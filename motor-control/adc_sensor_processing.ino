#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define CHARGE_PIN PB1
#define BTN_S1 PD2
#define BTN_S2 PD3
#define BTN_S3 PD4
#define BTN_S4 PD5
#define BTN_S5 PD6
#define ADC_CHANNEL 0
#define DEBOUNCE_MS 50
#define CUTOFF_FREQ 0.145 // Adjust as needed

volatile uint32_t millis_count = 0;
volatile uint8_t print_flag = 0;
char buffer[32];

// UART Function
void uart_init(uint16_t baud){
  uint16_t ubrr = F_CPU / 16 / baud -1;
  UBRR0H = (ubrr >> 8);
  UBRR0L = ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01} | (1 << UCSZ00);
}

void uart_transmit(char data){
  while(!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void uart_print(const char* str){
  while(*str) uart_transmit(*str++);
}

// ADC
void adc_init(){
  ADMUX = (1 << REFS0); // AVcc reference
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

uint16_t adc_read(uint8_t channel){
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

ISR(TIMER1_COMPA_vect){
  print_flag = 1;
}

ISR(TIMER0_COMPA_vect){
  millis_count++;
}

// IO init
void init_io(){
  DDRB |= (1 << CHARGE_PIN); // Output for charge/square wave
  PORTB &= ~(1 << CHARGE_PIN); // Default low
  DDRD &= ~((1 << BTN_S1) | (1 << BTN_S2) | (1 << BTN_S3) | (1 << BTN_S4) | (1 << BTN_S5));
  PORTD |=  (1 << BTN_S1) | (1 << BTN_S2) | (1 << BTN_S3) | (1 << BTN_S4) | (1 << BTN_S5)); // Pull-up
}

// Button debounce
uint8_t is_pressed_debounce(uint8_t pin){
  static uint32_t last_press_time[8] = {0};
  uint8_t pin_mask = (1 << pin);
  if(!(PIND & pin_mask)){
    if((millis_count - last_press_time[pin]) > DEBOUNCE_MS{
      if(!(PIND & pin_mask)){
        return 1;
      }
    }
  }
  return 0;
}

volatile uint8_t square_wave_running = 0;
volatile uint16_t square_wave_delay_ms = 0;
volatile uint32_t last_toggle_time = 0;

void square_wave_start(double freq){
  if(freq <= 0) return;
  square_wave_delay_ms = (uint16_t)(500.0 /freq);
  square_wave_running = 1;
  last_toggle_time = millis_count;
  PORTB &= ~(1 << CHARGE_PIN);
}

void square_wave_handler(){
  if(!square_wave_running) return;
  if((millis_count - last_toggle_time) >= square_wave_delay_ms){
    PORTB ^= (1 << CHARGE_PIN);
    last_toggle_time = millis_count;
  }
}

// Timer0 init for 1 ms tick
void timer0_init_1ms(){
  TCCR0A = (1 << WGM01); // CTC mode
  OCR0A = 249; // (16MHz/64/1000) -1 = 249
  TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
  TIMSK0 = (1 << OCIE0A); // Enable compare Interrupt
}

// Timer1 init for 50 ms voltage print
void timer1_init_50ms(){
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 3125; // 50 ms interval (16MHz/256/50 ms)
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12); // Prescaler 256
  TIMSK1 |= (1 << OCIE1A); // Enable Interrupt
}

void print_voltage(){
  uint16_t adc_val = adc_read(ADC_CHANNEL);
  uint16_t mv = (adc_val * 5000UL) /1023;
  snprintf(buffer, sizeof(buffer), "Voltage: %u.%02u V\r\n", mv /1000, (mv%1000)/10);
  uart_print(buffer);
}

void main(void) {
  cli();
  init_io();
  adc_init();
  uart_init(9600);
  timer0_init_1ms();
  timer1_init_50ms();
  uart_print("System Ready\r\n");
  sei();
  while(1){
    if(is_pressed_debounce(BTN_S1)){
      square_wave_running = 0; // Stop square wave
      PORTB |= (1 << CHARGE_PIN); // Steady 5V charge
    }
    else if(is_pressed_debounce(BTN_S2)){
      square_wave_start(CUTOFF_FREQ);
    }
    else if(is_pressed_debounce(BTN_S3)){
      square_wave_start(CUTOFF_FREQ /10);
    }
    else if(is_pressed_debounce(BTN_S4)){
      square_wave_start(CUTOFF_FREQ * 10);
    }
    else if(is_pressed_debounce(BTN_S5)){
       square_wave_running = 0; // Stop square wave
      PORTB &= ~(1 << CHARGE_PIN); // Discharge (0V)
    }
    square_wave_handler();
    if(print_flag){
      print_flag = 0;
      print_voltage();
    }
  }
}

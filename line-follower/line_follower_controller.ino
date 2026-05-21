#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
uint8_t Sensor1 = 0;
uint8_t Sensor2 = 1;
uint8_t POT = 7;
uint8_t BatteryLevel = 6;
uint16_t EEMEM eeprom_threshold;
uint16_t threshold = 500;

ISR(INT0_vect){
  _delay_ms(5);
  if(!(PIND & (1 << PD2))){
    int total = 0;
    for (int i = 0; i < 10; i++){
      uint16_t left = ReadADCSingleConversion(0);
      uint16_t right = ReadADCSingleConversion(1);
      threshold = (left + right) / 2;
      total += threshold;
    }
    threshold = total / 10;
    eeprom_update_word(&eeprom_threshold, threshold);
  }
}

void InitUSART(uint32_t baud_rate){
  uint16_t ubrr_val = (F_CPU - (8*baud_rate))/(16*baud_rate);
  UBRR0 = ubrr_val;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}


// Transmitting a single byte through USART
void TransmitByte(uint8_t data){
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

// Function to initialize ADC with specified channel
void InitADC(){
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  uint16_t garbage = ADC;
}

// Function to read ADC value from a given channel
uint16_t ReadADCSingleConversion(uint8_t channel){
  uint8_t MuxMask = (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);
  ADMUX &= ~MuxMask;
  ADMUX |= channel;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// Function to send string using transmit bytes
void printADC(char *str){
  while (*str){
    TransmitByte(*str++);
  }
}

// Initialising Timer 1
void InitTimer(){
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 15999;
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
  OCR1A = 0;
  OCR1B = 0;
}

void setM1speed(float motorSpeed){
  if(motorSpeed > 0){
    if(motorSpeed > 0 && motorSpeed <= 1.0){
      PORTB &= ~(1 << PB0);
      OCR1A = motorSpeed* ICR1;
    }
    else{
      PORTB &= ~(1 << PB0);
      OCR1A = 1.0* ICR1;
    }
  }
  else if(motorSpeed < 0){
    if(motorSpeed < 0 && motorSpeed >= -1.0){
      PORTB |= (1 << PB0);
      OCR1A = -motorSpeed* ICR1;
    }
    else{
      PORTB |= (1 << PB0);
      OCR1A = ICR1; // ICR1 is the TOP value
    }  
  }
  else{
    OCR1A =0;
  }
}

void setM2speed(float motorSpeed){
  if(motorSpeed > 0){
    if(motorSpeed > 0 && motorSpeed <= 1.0){
      PORTB &= ~(1 << PB3);
      OCR1B = motorSpeed* ICR1;
    }
    else{
      PORTB &= ~(1 << PB3);
      OCR1B = 1.0* ICR1;
    }
  }
  else if(motorSpeed < 0){
    if(motorSpeed < 0 && motorSpeed >= -1.0){
      PORTB |= (1 << PB3);
      OCR1B = -motorSpeed* ICR1;
    }
    else{
      PORTB |= (1 << PB3);
      OCR1B = ICR1; 
    }  
  }
  else{
    OCR1B =0;
  }
}

// Function to execute bang bang mode
void bangbangMode(){
  uint16_t POT_value = ReadADCSingleConversion(7);
  float set_speed_right = 0.8*((float)POT_value /1024);
  float set_speed_left = set_speed_right;
  uint16_t RightSensor_reading = ReadADCSingleConversion(0);
  uint16_t LeftSensor_reading = ReadADCSingleConversion(1);
  if (LeftSensor_reading > threshold && RightSensor_reading > threshold){ // move forward
    setM1speed(0.8*set_speed_right);
    setM2speed(0.8*set_speed_left);
  }
  else if (LeftSensor_reading < threshold && RightSensor_reading > threshold){ // turn right
    setM1speed(0);
    setM2speed(1.1*set_speed_left);
  }
  else if (LeftSensor_reading > threshold && RightSensor_reading < threshold){ // turn left
    setM1speed(1.1*set_speed_right);
    setM2speed(0);
  }
  else if (LeftSensor_reading < threshold && RightSensor_reading < threshold){ // going backwards
    setM1speed(-0.3*set_speed_right);
    setM2speed(-0.3*set_speed_left);
  }
}

void LineFollow_PMode(){
  uint16_t POT_value = ReadADCSingleConversion(POT);
  float base_speed = (float)POT_value /1024.0;
  float set_speed_left = 0.7 * base_speed;
  float set_speed_right = 0.7 * base_speed;
  uint16_t left_read = ReadADCSingleConversion(Sensor1);
  uint16_t right_read = ReadADCSingleConversion(Sensor2);
  int16_t error = right_read - left_read;
  float abs_error = abs(error);
  float kKp = 0.005;
  float offset = kKp * error;
  if(offset > 1.0) offset = 1.0;
  if(offset < -1.0) offset = -1.0;
  // Calculate adjusted speed
  float left_speed = (1.0 - offset)*set_speed_left;
  float right_speed = (1.0 + offset)*set_speed_right;
  // Limiting speed
  if(left_speed > 1.0) left_speed = 0.7;
  if(left_speed < 0.0) left_speed = -0.2;
  if(right_speed > 1.0) left_speed = 0.7;
  if(right_speed < 0.0) left_speed = -0.2;
  // Reverse
  if(abs(error)<250 && left_read < threshold && right_read < threshold){
    setM1speed(-0.5 * set_speed_right);
    setM2speed(-0.5 * set_speed_left);
  }
  else{
    setM1speed(right_speed);
    setM2speed(left_speed);
  }
}

int main(void){
  PORTD |= (1 << PD4) | (1 << PD5) | (1 << PD2);
  InitUSART(9600);
  InitTimer();
  InitADC();
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC01);
  sei();
  threshold = eeprom_read_word(&eeprom_threshold);
  while(1){
    uint8_t BatteryP = 6;
    uint16_t battery_adc = ReadADCSingleConversion(BatteryP);
    float battery_voltage = (battery_adc * 5.0) / 1024.0;
    float low_battery_threshold = 2.5;
    if(battery_voltage < low_battery_threshold){
      setM1speed(0);
      setM2speed(0);
      PORTD |= (1 << PB5);
      char message[] = "Low Battery!\r\n";
      printADC(message);
    }
    else{
      PORTD &= ~(1 << PB5);
    }
    uint8_t checkD4 = !(PIND & (1 << PD4));

    // Checking which mode to be used
    if(!checkD4){
      bangbangMode();
    }
    if(checkD4){
      LineFollow_PMode();
    }
    _delay_ms(10);
  }
  return 0;
}

const int timing = 180;
void setup(){
  DDRC = 11111111;
  DDRB = 11111111;
}

void loop(){
  PORTC = 11111111;
  PORTB = 11111111;
  delay(timing/4);
  PORTC = 00000000;
  PORTB = 00000000;
  delay(timing);
}

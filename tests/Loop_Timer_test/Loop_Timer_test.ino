unsigned long loop_timer;  // loop timer to maintain 250Hz refresh rate
unsigned short REFRESH_RATE_MICROS = 4000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(micros() <=  loop_timer + REFRESH_RATE_MICROS);
  loop_timer = micros();  
}

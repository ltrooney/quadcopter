// Battery voltage: 9V-12.6V
// 9V-12.6V corresponds to 3.516-4.922V read by Arduino
// Voltage divider uses resistors with values 1560 ohm & 1K ohm.
// The status LED will turn on when the battery voltage reaches 9.5V 
// which is converted to 3.711V by voltage divider.

// designate pinouts
const int PIN_LOW_VOLT_MON = A0;
const int PIN_LOW_VOLT_STATUS = 13;

// constants
//const float VOLTAGE_MIN = 3.711;  // 9.5V battery voltage
const float VOLTAGE_MIN = 2.25;

void setup() {
  // setup pinouts
  Serial.begin(9600);
  pinMode(PIN_LOW_VOLT_STATUS, OUTPUT);
}

void loop() {
  // read voltage as value between 0-1023
  int sensorValue = analogRead(PIN_LOW_VOLT_MON);

  // convert sensorValue to voltage between 0V-5V
  float voltage = sensorValue * (5.0 / 1023.0);

  if(voltage < VOLTAGE_MIN) {
    digitalWrite(PIN_LOW_VOLT_STATUS, HIGH);
    Serial.println("Low Voltage");
  } else {
    digitalWrite(PIN_LOW_VOLT_STATUS, LOW);
  }
  Serial.println(voltage);
  delay(1000);
}

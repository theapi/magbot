/*
 Simple Temperature uses the lm35 in the basic centigrade temperature configuration
*/

int sensor_pin = 0; // analog input pin
int interval = 1000; // How often we take a reading

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // Gets and prints the raw data from the lm35
  int val = analogRead(sensor_pin);
  Serial.print("RAW DATA: ");
  Serial.print (val);
  Serial.println(" ");
  
  // Convert the raw data into degrees celsius and print it out
  // 500mV/1024 = 0.48828125
  float temperature = val * 0.48828125;
  Serial.print("CELSIUS: ");
  Serial.print(temperature);
  Serial.println("*C ");
  
  delay(interval);
}


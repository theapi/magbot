/*
 Simple Temperature uses the lm35 and outputs both Celsius & Fahrenheit
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
  
  // Convert celsius into fahrenheit 
  temperature = temperature *9 / 5;
  temperature = temperature + 32;
  Serial.print("FAHRENHEIT: ");
  Serial.print(temperature);
  Serial.println("*F");
  
  delay(interval);
}


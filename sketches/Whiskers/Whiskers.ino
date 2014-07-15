/**
 Whiskers for Magbot.
*/

// Whiskers configuration
const byte whiskers_horiz = A5; // The analog input pin
const byte whiskers_vert = A4; // The analog input pin

const byte whiskers_delay = 100; // How often to check the whiskers (milliseconds).
const byte whiskers_threshold = 5; // How much the whiskers reading is allowed to fluctuate.
int whiskers_vert_default = 512; // Assume half of full analogRead.
int whiskers_horiz_default = 512; // Assume half of full analogRead.

void setup() {
  
  // Open serial monitor at 9600 baud to see debugging messages.
  Serial.begin(9600); 
  
  // Remember where center is.
  whiskersCalibrate();
}

void loop() {
  
  whiskersCheck();
  delay(whiskers_delay);

}

/********************************************************************************
Whiskers functions
********************************************************************************/

void whiskersCalibrate()
{
  whiskers_horiz_default = analogRead(whiskers_horiz);
  whiskers_vert_default = analogRead(whiskers_vert);
  Serial.print("whisker calibration: ");
  Serial.print(whiskers_vert_default);
  Serial.print(" : ");
  Serial.println(whiskers_horiz_default);
}



void whiskersCheck()
{

  // If the reading of the thumbstick is not the same as when calibrated,
  // we've hit something.
  
  // Vertical check
  int val = analogRead(whiskers_vert);
  if (val > whiskers_vert_default + whiskers_threshold || val < whiskers_vert_default - whiskers_threshold) {
    Serial.println("BUMP VERTICAL!");
    Serial.print(whiskers_vert_default);
    Serial.print(" default - whisker read: ");
    Serial.println(val);

    
  } else {
  
    // Horizontal check
    int val = analogRead(whiskers_horiz);
    
    if (val > whiskers_horiz_default + whiskers_threshold) {
      Serial.println("BUMP LEFT!");
      Serial.println(whiskers_horiz_default);
      Serial.print(" default - whisker read: ");
      Serial.println(val);
    
    } else if (val < whiskers_horiz_default - whiskers_threshold) {
      Serial.println("BUMP RIGHT!");
      Serial.println(whiskers_horiz_default);
      Serial.print(" default - whisker read: ");
      Serial.println(val);
    } 
  }

}


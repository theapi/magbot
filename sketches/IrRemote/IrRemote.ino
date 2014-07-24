/*
 * IrRemote: demonstrates translating the remote codes to a human readable format.
 */

// https://github.com/shirriff/Arduino-IRremote
#include <IRremote.h>

// the pin used for the infrared receiver 
int RECV_PIN = 11;

// Create an instance of the IRrecv library
IRrecv irrecv(RECV_PIN);

// Structure containing received data
decode_results results;

// Used to store the last code received. Used when a repeat code is received
unsigned long LastCode;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  // Check for a new IR code
  if (irrecv.decode(&results)) {
    //Serial.println(results.value, HEX);
    
    // Cet the button name for the received code
    irHandleInput(results.value);
    // Start receiving codes again
    irrecv.resume();
  }
}


/********************************************************************************
IR remote functions
********************************************************************************/

/* Function returns the button name relating to the received code */
void irHandleInput(unsigned long code)
{
  
  /* Character array used to hold the received button name */
  char CodeName[3];
  /* Is the received code is a repeat code (NEC protocol) */
  if (code == 0xFFFFFFFF)
  {
    /* If so then we need to find the button name for the last button pressed */
    code = LastCode;
  }
  /* Save this code incase we get a repeat code next time */
  LastCode = code;
  
  /* Find the button name for the received code */
  switch (code)
  {
    /* Received code is for the POWER button */
  case 0xFFA25D:
    strcpy (CodeName, "PW");
    break;
    /* Received code is for the MODE button */
  case 0xFF629D:
    strcpy (CodeName, "MO");
    break;
    /* Received code is for the MUTE button */
  case 0xFFE21D:
    strcpy (CodeName, "MU");
    break;
    /* Received code is for the REWIND button */
  case 0xFF22DD:
    strcpy (CodeName, "RW");
    break;
    /* Received code is for the FAST FORWARD button */
  case 0xFF02FD:
    strcpy (CodeName, "FW");
    break;
    /* Received code is for the  PLAY/PAUSE button */
  case 0xFFC23D:
    strcpy (CodeName, "PL");
    break;
    /* Received code is for the EQ button */
  case 0xFFE01F:
    strcpy (CodeName, "EQ");
    break;
    /* Received code is for the VOLUME - button */
  case 0xFFA857:
    strcpy (CodeName, "-");
    break;
    /* Received code is for the VOLUME + button */
  case 0xFF906F:
    strcpy (CodeName, "+");
    break;
    /* Received code is for the number 0 button */
  case 0xFF6897:
    strcpy (CodeName, "0");
    break;/* Received code is for the RANDOM button */
  case 0xFF9867:
    strcpy (CodeName, "RN");
    break;
    /* Received code is for the UD/SD button */
  case 0xFFB04F:
    strcpy (CodeName, "SD");
    break;
    /* Received code is for the number 1 button */
  case 0xFF30CF:
    strcpy (CodeName, "1");
    break;
    /* Received code is for the number 2 button */
  case 0xFF18E7:
    strcpy (CodeName, "2");
    break;
    /* Received code is for the number 3 button */
  case 0xFF7A85:
    strcpy (CodeName, "3");
    break;
    /* Received code is for the number 4 button */
  case 0xFF10EF:
    strcpy (CodeName, "4");
    break;
    /* Received code is for the number 5 button */
  case 0xFF38C7:
    strcpy (CodeName, "5");
    break;
    /* Received code is for the number 6 button */
  case 0xFF5AA5:
    strcpy (CodeName, "6");
    break;
    /* Received code is for the number 7 button */
  case 0xFF42BD:
    strcpy (CodeName, "7");
    break;
    /* Received code is for the number 8 button */
  case 0xFF4AB5:
    strcpy (CodeName, "8");
    break;
    /* Received code is for the number 9 button */
  case 0xFF52AD:
    strcpy (CodeName, "9");
    break;
    /* Received code is an error or is unknown */
  default:
    strcpy (CodeName, "??");
    break;
  }

  Serial.println(CodeName);
}


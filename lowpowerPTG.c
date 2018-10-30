#include "Arduino.h"
// avr - Version: Latest
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
// serial library
#include <SoftwareSerial.h>
SoftwareSerial Serial2(8,9); //PM2.5

// Power SIM
#define POWERSIM (11)
// Power Switch
#define POWERSW (12)
// Power Sensor
#define POWER (13)
// Analog Port
#define LEVEL_SENSOR A3

float level;
float voltage;
char aux_str[270];
unsigned long InitialTime = millis();


//const unsigned long POSTinterval = 1000UL*60; //one minute interval
//unsigned long POSTtimer;

// This variable is made volatile because it is changed inside
// an interrupt function
volatile int f_wdt=1;
int count;
// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) {
	if(f_wdt == 0) {
		// here we can implement a counter the can set the f_wdt to true if
		// the watchdog cycle needs to run longer than the maximum of eight
		// seconds.
		//int count; otherwise, we are initializing this variable, to 0, every time ISR is called
		//Serial.println(count);
		if(count == 15){//how much time, in sleep mode, equal 1 count?
		  f_wdt=1;
		  count=0;
		}
		else
		  count++;
	}
}

// Enters the arduino into sleep mode.
void enterSleep(void)
{
	// There are five different sleep modes in order of power saving:
	// SLEEP_MODE_IDLE - the lowest power saving mode
	// SLEEP_MODE_ADC
	// SLEEP_MODE_PWR_SAVE
	// SLEEP_MODE_STANDBY
	// SLEEP_MODE_PWR_DOWN - the highest power saving mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	// Now enter sleep mode.
	sleep_mode();
	// The program will continue from here after the WDT timeout
	// First thing to do is disable sleep.
	sleep_disable();
	// Re-enable the peripherals.
	power_all_enable();

        memset(aux_str, '\0', 270);
}

// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {

	// The MCU Status Register (MCUSR) is used to tell the cause of the last
	// reset, such as brown-out reset, watchdog reset, etc.
	// NOTE: for security reasons, there is a timed sequence for clearing the
	// WDE and changing the time-out configuration. If you don't use this
	// sequence properly, you'll get unexpected results.

	// Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
	MCUSR &= ~(1<<WDRF);

	// Configure the Watchdog timer Control Register (WDTCSR)
	// The WDTCSR is used for configuring the time-out, mode of operation, etc

	// In order to change WDE or the pre-scaler, we need to set WDCE (This will
	// allow updates for 4 clock cycles).

	// Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
	// bit must be set in order to change WDE or the watchdog pre-scalers.
	// Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
	// clock cycles then it will be reset by hardware.
	WDTCSR |= (1<<WDCE) | (1<<WDE);

	/**
	 *	Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
	 *	WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
	 *	0    0    0    0    |   2K cycles   | 16 ms
	 *	0    0    0    1    |   4K cycles   | 32 ms
	 *	0    0    1    0    |   8K cycles   | 64 ms
	 *	0    0    1    1    |  16K cycles   | 0.125 s
	 *	0    1    0    0    |  32K cycles   | 0.25 s
	 *	0    1    0    1    |  64K cycles   | 0.5 s
	 *	0    1    1    0    |  128K cycles  | 1.0 s
	 *	0    1    1    1    |  256K cycles  | 2.0 s
	 *	1    0    0    0    |  512K cycles  | 4.0 s
	 *	1    0    0    1    | 1024K cycles  | 8.0 s
	*/
	WDTCSR  = (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
	// Enable the WD interrupt (note: no reset).
	WDTCSR |= _BV(WDIE);
}

void setup() {
    Serial.begin(9600);
    Serial2.begin(9600);
	  // Set PIN modes
	  pins_init();
	  Serial.println("Initialising...");
	  delay(100);
	  setupWatchDogTimer();
    InitialTime = millis();
    Serial.print("Initial Time in Seconds: ");
    Serial.println((InitialTime)/1000);

    Serial.println("Initialisation complete.");
	  delay(100);
}

void pins_init()
{
  pinMode(LEVEL_SENSOR, INPUT);
  pinMode(POWER, OUTPUT);
  pinMode(POWERSIM, OUTPUT);
  pinMode(POWERSW, OUTPUT);

}
// Get raw sensor value
int getMaxValue()
  {
    int sensorValue;

    int sensorMax = 0;

    uint32_t start_time = millis();

    while ((millis()-start_time)<100) // sensor reading gets attenuated if sample <= 10ms, WHY?
    {
      sensorValue = analogRead(LEVEL_SENSOR);

      if(sensorValue > sensorMax)
      {
        sensorMax = sensorValue;
      }
    }
    return sensorMax;
  }
// Power ON hall sensor
void powerONSensor()
  {
  digitalWrite (POWER, LOW);  // turn power ON
  delay(1); // give time to power up
  }  // end of powerONSensor

// Power OFF hall sensor
void powerOFFSensor()
  {
  digitalWrite (POWER, HIGH);  // turn power OFF
  delay(1); // give time to power down
  }  // end of powerOFFSensor

// Power ON Boost
void powerONBoost()
  {
  digitalWrite (POWERSW, LOW);  // turn power ON
  delay(1); // give time to power up
  }  // end of powerON SIM

// Power OFF Boost
void powerOFFBoost()
  {
  digitalWrite (POWERSW, HIGH);  // turn power OFF
  delay(1); // give time to power down
  }  // end of powerOFF SIM

// Power ON SIM
void powerONSIM()
  {
  digitalWrite (POWERSIM, HIGH);  // turn transistor ON
  delay(1); // give time to power up
  }  // end of powerON SIM

// Power OFF SIM
void powerOFFSIM()
  {
  digitalWrite (POWERSIM, LOW);  // turn transistor OFF
  delay(1); // give time to power down
  }  // end of powerOFF SIM

enum SIM
    {
      START,SENSOR,
      PWRSIM,ATTACHGPRS,
      OPENGPRS,QUERYGPRS,
      INITHTTP,SENDPARA,
      ADDLEVEL,CLOSEURL,
      ENDPARA,ENDGPRS,
      SIMOFF
    };

SIM SIM_state  = START;
uint32_t stateTime;


void loop() {
int countloop=0;
char charVal[] = "00.00";
char variable;

  switch (SIM_state)
  {
    case START:

    	if(f_wdt != 1){  //Wait here indefenetly until a flag is rised; TRUE if f_wdt = 0
    	  //SIM_state = START;
    	  //break;
		    return;
	    }

	      SIM_state = SENSOR; // wake-up if f_wdt = 1
      break;

    //Get reading from Hall sensor
    case SENSOR:
    	InitialTime = millis();
    	//Serial.print("Initial Time in MilliSeconds: ");
    	//Serial.println(InitialTime);
    	Serial.println(F("----- Case SENSOR -----"));
      int sensor_max;
      memset(charVal, '\0', sizeof(charVal));
      // Sensor ON
      powerONSensor();
	    sensor_max = getMaxValue();
      voltage = (sensor_max / 1024.0) * 5000; // Gets you voltage in mV
      level = (voltage/5000) * 100 ; // Get you level in %
      Serial.print(F("---> Level: "));
      Serial.println(level);
      dtostrf(level, 5, 2, charVal);
      Serial.print(F("---> Level(as char): "));

      for(int i=0; i<sizeof(charVal); i++){
				Serial.print(charVal[i]);
			}
     	Serial.println();
      //Serial.println(level,5);// the '5' after level allows you to display 5 digits after decimal point
	    // wait, certain amount of time, then OFF sensor
	    delay(100);
	    // Sensor OFF
	    powerOFFSensor();
	    SIM_state = PWRSIM;
      break;

    case PWRSIM:
	    Serial.println(F("----- Case PWRSIM -----"));
	      //Power Boost Switch ON
	      powerONBoost();
	      //PowerSIM ON
	      powerONSIM(); //High pulse 1.2 seconds, makes Vc = 0 for 1.2 seconds
	      delay(1200);
	      powerOFFSIM(); // Vc=VBAT

	      sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications

	      countloop=0;
	      while( (sendATcommand("AT+CREG?", "+CREG: 1,1", 600) // before 0,1
	            || sendATcommand("AT+CREG?", "+CREG: 0,5", 600)
	            || sendATcommand("AT+CREG?", "+CREG: 0,1", 600)
	            || sendATcommand("AT+CREG?", "+CREG: 2,1", 600)) == 0 ){
								countloop++;
								if(countloop >= 8){ //Why 8 countloops?
									break;
								}
							};

	      sendATcommand("AT+CREG=0", "OK", 5000);

	     if(countloop < 8){
				 SIM_state = ATTACHGPRS;
				 Serial.println(F("Connected to the network!"));
			 }
	     else {
				 SIM_state = PWRSIM;
				 Serial.println(F("NOT Connected to the network!"));
			 }

	     break;

    case ATTACHGPRS:
	    memset(aux_str, '\0', 270);
	    Serial.println(F("----- Case ATTACHGPRS -----"));
	    sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
	    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
	    sendATcommand(aux_str, "OK", 2000);
	    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
	    sendATcommand(aux_str, "OK", 2000);
	    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
	    sendATcommand(aux_str, "OK", 2000);

	    countloop=0;

	    while ( (  sendATcommand("AT+SAPBR=1,1", "OK", 6000)
	            || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 6000) ) != 1){
							 countloop++;
							 if(countloop >= 10){
								 break;
							 }
						 };
	    if(countloop < 5){
	      SIM_state = INITHTTP;
	      Serial.println(F("Connected to the Internet!"));
			}
	    else {
	      SIM_state = ATTACHGPRS;
	      Serial.println(F("NOT Connected to the Internet!"));
			}
	    break;

    case INITHTTP:
    	Serial.println(F("----- Case INITHTTP -----"));
      countloop=0;
      while (sendATcommand("AT+HTTPINIT", "OK", 1000) != 1)
			{
				countloop++;
				if(countloop >= 3){
					break;
				}
			}; //why (;) ?
      delay(100);
    	sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 10000);

	    if(countloop < 3){ //Why 3 countloop?
	      SIM_state = SENDPARA;
	      Serial.println(F("Init Http functional!"));
			}
	      else {
	      SIM_state =  INITHTTP;
	      Serial.println(F("Init Http NOT functional!"));
			}
      break;

    case SENDPARA:
     Serial.println(F("----- Case SENDPARA -----"));
     snprintf(aux_str, sizeof(aux_str), "AT+HTTPPARA=\"URL\",\"http://www.smability.com/airquality/airdata.php?deviceID=04%sreading=%s%salarm=00%stype=%s\"","&",charVal,"&","&","temperature");
     sendATcommand(aux_str, "OK", 520);

     countloop=1;
     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,34", 11000) != 1){ //45 is the lenght of char text from file csv2sql
			 delay(3);
			 if(countloop >= 120){ //Why 120 countloop?
					break;
			};
			countloop++;
		};

     if(countloop >=120 ){
         Serial.println(F("Failure in sending data"));
         SIM_state = SENDPARA;
         //evilcounter=evilcounter+1;
     }
		 else{
         Serial.println(F("Success in sending data"));
         if(sendATcommand("AT+HTTPREAD", "!!", 1200)) // Move inside
         {
           Serial.println(F("Success in handshaking"));
           SIM_state = ENDGPRS;
           //evilcounter=0;
           // Aqui parpadeamos los leds
         }
     }
     break;

    case ENDGPRS:
    	Serial.println(F("----- Case ENDGPRS -----"));
      sendATcommand("AT+CLTS=0", "OK", 2000); // "Get Local Time" Stamp Disabled
      sendATcommand("AT+HTTPACTION=0", "OK", 2000);
      sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
      sendATcommand("AT+HTTPTERM", "OK", 4000); // closing http if open
      SIM_state = SIMOFF;
    	break;

    case SIMOFF: //  Sleep Mode
    	Serial.println(F("----- Case SIMOFF -----"));
    	Serial.println();

      //PowerSIM OFF!
      powerONSIM(); //Vc=low pulse 1.2 seconds
      delay(1200);
      powerOFFSIM(); //Vc=VBAT
      //power Switch OFF
      powerOFFBoost();
      SIM_state = START; // Start over

      f_wdt = 0;// Clear the flag so we can run above code again after the MCU wake up

      //Serial.print("Elapsed Time in MilliSeconds: ");
      //Serial.println((millis() - InitialTime));

      Serial.println(F("----- ----- -----"));
      Serial.println();

      enterSleep(); // Re-enter sleep mode.

      break;
  }
}

/////////////////////////////////////////////////////////////

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[270];
    unsigned long previous;

    memset(response, '\0', 270);    // Initialize the string

    delay(10);

    while( Serial2.available() > 0)
			Serial2.read();    						// Clean the input buffer

    Serial2.println(ATcommand);    // Send the AT command

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(Serial2.available() != 0){
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = Serial2.read();
            delay(5);
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL)
            {
                answer = 1;
            }
            if (strstr(response, "+FTPPUT: 1,6") != NULL)
            {
              Serial.println(F("FTP Error Code"));
                answer = 2;
            }
            if (strstr(response, "+HTTPACTION: 0,6") != NULL)
            {
              Serial.println(F("HTTP Error Code"));
                answer = 2;
            }
            if (strstr(response, "ERROR") != NULL)
            {
              Serial.println(F("AT Command ERROR"));
                answer = 2;
            }
            if (strstr(response, "Location Not Fix") != NULL)
            {
              Serial.println(F("GPS not found"));
                answer = 2;
            }
            /*if (strstr(response, "+CREG: 0,2") != NULL)
            {
              Serial.println(F("Network not found"));
                answer = 2;
            }*/
        }
    }
    // Waits for the asnwer with time out
    while((answer == 0) && ((millis() - previous) < timeout));
			Serial.println(response);    // Send the AT command

    if((millis() - previous) > timeout){
        Serial.print(F("Time exceeded: "));
        Serial.println(millis() - previous);
    }
    return answer;
}


#include "DHT.h"
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <stdio.h> 
#include <string.h> 
#include <math.h>

//                     rx,tx
SoftwareSerial SerialPM(12,13);  //  PM2.5
SoftwareSerial SerialSIM(10,11);  //  SIM // revisar
//Serial1 --> Ozone
Adafruit_GPS GPS(&Serial2);
//Serial3 --> CO


#define DHTPIN A12     
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

#define BatteryVoltage A14  // reading battery voltage here  

#define DataLed 37// output led // revisar
#define GPSLed 33 // output led // revisar
#define BattLed 7// output led // revisar

#define GPSInput 43 // input switch ON
#define DCSupply 8 //input signal
#define SwitchON A7 // input switch 

#define SIMPower 4  //output signal
#define GPSPower 5  //output signal // revisar

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long currentMillis = 60000; 
unsigned long previoustest = 0;

boolean GPSFlag=false; // true means GPS has been found before
boolean InternetEnabled=0;
uint8_t evilcounter=0;
uint8_t countloop=0;      

boolean StartLecture=true; // yyy... for debug
float BattVolt=4.0; // YYY... 

// 25.709111,-100.303167

char GPStext1[]="25.709111";
char GPStext2[]="-100.303167";
char GPStext3[23]; // 

double latitude=19.401625;
double longitude=-100.123456;
int sensorValue = 0; // variable to store the value coming from the sensor  
char test_str[ ]="100\n";

int ValuePm10=1000; 

boolean DevelopmentMode = false; // echo of lines
boolean testmode=false; // geolocalization

char charVal[5];

float oldGPSlat=0;
float oldGPSlon=0;
double dummyGPS=0.0001;

void setup(){
  
  delay(1000);
    
  Serial.begin(19200); //Display

  pinMode(BattLed, OUTPUT);     //Status Batt  
  pinMode(GPSLed, OUTPUT);     //Status GPS
  pinMode(DataLed, OUTPUT);     //Status GPS
  
  pinMode(DCSupply, INPUT);     //HIGH = connected 
  pinMode(GPSInput, INPUT);     //HIGH = connected 
  pinMode(SwitchON, INPUT);     //HIGH = connected 
 
  pinMode(BatteryVoltage, INPUT);     //HIGH = connected 
    
  pinMode(SIMPower, OUTPUT);     //Power ON,OFF, command to turn the sim808 //PWR KEY
  digitalWrite(SIMPower, LOW);  //Always low

  pinMode(GPSPower, OUTPUT);     //Power ON,OFF, command to turn the sim808 //PWR KEY
  //digitalWrite(GPSPower, LOW);  //Low to keep ON
  //digitalWrite(GPSPower, HIGH);  //Always high to keep OFF 
 
  SerialPM.begin(9600); //PM2.5 X
  Serial1.begin(9600);  //O3  
  GPS.begin(9600);  // GPS  
  Serial3.begin(9600);  //CO   
  SerialSIM.begin(9600); // SIM X
  dht.begin();  
    
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Battery Voltage!!!"));
    Serial.println(F("---------------------------------------"));
    
    BattVolt=analogRead(BatteryVoltage)*(2*5.0/1023); //Voltage divider
    Serial.print(F("Battery Voltage: ")); 
    Serial.print(analogRead(BatteryVoltage));
    Serial.print(F(" bits (of 1023) = "));
    Serial.print(BattVolt);
    Serial.println(F(" V"));
    //Serial.println(F("Note: Undervoltage warning if Vbat=3.54 V (Cap aprox.: 18/100)"));
    
    Serial.print(F("DC Supply Connected: "));
    Serial.println(digitalRead(DCSupply));

    Serial.print(F("GPS ON: "));
    Serial.println(digitalRead(GPSInput));

    Serial.print(F("Switch ON: "));
    Serial.println(digitalRead(SwitchON));
       
///////////////////////////////////
    
    Serial.println(F("---------------------------------------"));
    Serial.println(F("PM2.5 Start!!!"));
    Serial.println(F("---------------------------------------"));
    SerialPM.listen();
    mide25(1,0);  //Stop PM2.5
    delay(50);
    mide25(2,5);
    delay(50);

    Serial.println(F("---------------------------------------"));
    Serial.println(F("CO Start!!!"));
    Serial.println(F("---------------------------------------"));
  
    MeasureCO(1);
    MeasureCO(2);

    Serial.println(F("---------------------------------------"));
    Serial.println(F("O3 Start!!!"));
    Serial.println(F("---------------------------------------"));
  
    MeasureO3(1);
    MeasureO3(2);

    //=================================================  
      
    while(!digitalRead(SwitchON))
    {BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000); }

    if (digitalRead(DCSupply)){
      if(BattVolt<4.00){
        for(int mm=1;mm<=5;mm++){BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);} //3 seg por ciclo
        };
      while(BattVolt<4.00) // Antes 3.75 
      {
        BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);   
        BattVolt=analogRead(BatteryVoltage)*(2*5.0/1023); //Voltage divider
        Serial.print(F("Battery Voltage: ")); 
        Serial.print(analogRead(BatteryVoltage));
        Serial.print(F(" bits (of 1023) = "));
        Serial.print(BattVolt);
        Serial.println(F(" V")); 
      }
    }
    
    while(!digitalRead(SwitchON))
    {BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000); }

    if (digitalRead(DCSupply)){
      if(BattVolt<4.00){
        for(int mm=1;mm<=5;mm++){BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);} //3 seg por ciclo
        };
      while(BattVolt<4.00) // Antes 3.75 
      {
        BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);   
        BattVolt=analogRead(BatteryVoltage)*(2*5.0/1023); //Voltage divider
        Serial.print(F("Battery Voltage: ")); 
        Serial.print(analogRead(BatteryVoltage));
        Serial.print(F(" bits (of 1023) = "));
        Serial.print(BattVolt);
        Serial.println(F(" V")); 
      }
    }

    while(!digitalRead(SwitchON))
    {BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000); }

    if (digitalRead(DCSupply)){
      if(BattVolt<4.00){
        for(int mm=1;mm<=5;mm++){BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);} //3 seg por ciclo
        };
      while(BattVolt<4.00) // Antes 3.75 
      {
        BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);   
        BattVolt=analogRead(BatteryVoltage)*(2*5.0/1023); //Voltage divider
        Serial.print(F("Battery Voltage: ")); 
        Serial.print(analogRead(BatteryVoltage));
        Serial.print(F(" bits (of 1023) = "));
        Serial.print(BattVolt);
        Serial.println(F(" V")); 
      }
    }

    while(!digitalRead(SwitchON))
    {BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000); }

    if (digitalRead(DCSupply)){
      if(BattVolt<4.00){
        for(int mm=1;mm<=5;mm++){BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);} //3 seg por ciclo
        };
      while(BattVolt<4.00) // Antes 3.75 
      {
        BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);   
        BattVolt=analogRead(BatteryVoltage)*(2*5.0/1023); //Voltage divider
        Serial.print(F("Battery Voltage: ")); 
        Serial.print(analogRead(BatteryVoltage));
        Serial.print(F(" bits (of 1023) = "));
        Serial.print(BattVolt);
        Serial.println(F(" V")); 
      }
    }

    //=================================================    
    
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Leds Starting!!!"));
    Serial.println(F("---------------------------------------"));
    
    BlinkLeds(1);
    
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Restarting SIM!!!"));
    Serial.println(F("---------------------------------------"));

    SerialSIM.listen();
    BlinkLeds(1); 
    // close_3G(); 
    // power_off(); 
    BlinkLeds(1); 
    power_on();
    InternetEnabled=configure_3G(); 
    if(InternetEnabled){digitalWrite(DataLed,1);}

    
    if (digitalRead(GPSInput)) 
    {
      Serial.println(F("---------------------------------------"));
      Serial.println(F("GPS Start"));
      Serial.println(F("---------------------------------------"));
      digitalWrite(GPSPower, LOW);  //Low to keep ON
      delay(1000);
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      GPS.sendCommand(PGCMD_ANTENNA);     
      useInterrupt(true);
      BlinkLeds(3);
      Serial.println(F("---------------------------------------"));
      Serial.println(F("GPS Search!!!"));
      Serial.println(F("---------------------------------------"));  
      GPSFlag=SearchGPS();
      Serial.print(F("GPS Flag: "));
      Serial.println(GPSFlag);
    }else{
      digitalWrite(GPSPower, HIGH);  //High to keep OFF 
    } 
    Serial.println(F("---------------------------------------"));   
    Serial.println(F("First measurement & loop!!!"));
    Serial.println(F("---------------------------------------"));  

}


void loop(){

char charVal[5];

char CharT[6]; // [*C] 4
char CharH[6]; // [%RH] 4
char CharBatt[3]; // [%] 3
char CharO3[4]; // [ppb] 4
char CharCO[5]; // [ppb] 5
char CharPm10[4]; // [ug/m3] 4
char CharPm25[4]; // [ug/m3] 4
char CharMode[1]; // [-] 1
//char GPStext3[20]; // [-] 20
char CharDev[4]; //[1] 4

char str_temp[6];
char str_temp2[10];

float bufferT[4]={0,0,0,0};
float bufferH[4]={0,0,0,0};
int bufferBatt[4]={0,0,0,0};
int bufferCO[4]={0,0,0,0};
int bufferO3[4]={0,0,0,0};
int bufferPM25[4]={0,0,0,0};
int bufferPM10[4]={0,0,0,0};

unsigned int answer1=0;
float answerf=0;

/////////////// Battery analysis 

BattVolt=(2*5.0/1023)*analogRead(BatteryVoltage);

while(!digitalRead(SwitchON))
{BlinkLeds(4);digitalWrite(DataLed, LOW);digitalWrite(GPSLed, LOW);delay(2000);
InternetEnabled=false;GPSFlag=false;}

if (digitalRead(DCSupply))
{
   // if (BattVolt<4.25){digitalWrite(BattLed, HIGH);} //4.30 anterior
   // else{digitalWrite(BattLed, LOW);} 
   digitalWrite(BattLed, LOW);
   if(digitalRead(GPSInput)){BlinkLeds(4);}
   else{delay(2000);BlinkLeds(4);}   
}else{
  if (BattVolt<3.55){digitalWrite(BattLed, HIGH);} // 3.53 = 20%, 3.55 = 23% 
  else {digitalWrite(BattLed, LOW);} 
}
   
/////////////// Playing with GPS during execution

if(InternetEnabled) 
{
  digitalWrite(DataLed,1);
  
  if(digitalRead(GPSInput))
  {    
      if(!GPSFlag){
      BlinkLeds(3); // blue
      digitalWrite(GPSPower, LOW); //Low to keep ON
      delay(1000);

      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      GPS.sendCommand(PGCMD_ANTENNA);
      
      useInterrupt(true);
      GPSFlag=SearchGPS();
      digitalWrite(GPSLed,GPSFlag);
      }else{
        GPSFlag=SearchGPS();
        digitalWrite(GPSLed,GPSFlag); 
        // yyy put here search gps??? below does it find it??
      }
  }else{ 
      digitalWrite(GPSPower, HIGH); //High to keep OFF
      digitalWrite(GPSLed,0);
      if(GPSFlag){
        BlinkLeds(3); // blue
        GPSFlag=0;
      }
  }
}else{
   digitalWrite(DataLed,0);
   SerialSIM.listen();
   close_3G(); 
   delay(1000);
   power_on();
   InternetEnabled=configure_3G();
   GPSFlag=0;
   StartLecture=true; 
   MeasureO3(1);
   MeasureO3(2);
   MeasureCO(1);
   MeasureCO(2);
}

currentMillis = millis(); 

if(digitalRead(GPSInput)){
  
  previoustest = millis();
  
   ////////////////////OUTDOOR routine ////////////// 
    if ((currentMillis - previousMillis >= 6000) && GPSFlag && InternetEnabled ) { 
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Measurement aprox. every 8 secs..."));
    Serial.println(F("---------------------------------------"));
    
    previousMillis = currentMillis;
  
////////////////////Battery value ////////////// 

   Serial.println(F("---------------------------------------"));
   Serial.println(F("Battery Level!!!"));
   Serial.println(F("---------------------------------------"));
   SerialSIM.listen();
   sensorValue=0;
   memset(CharBatt, '\0', sizeof(CharBatt));

   sendATcommand("AT+CBC", "OK", 5000);
   sendATcommand("AT+CBC", "OK", 5000);
   sensorValue=RetrieveBatt("AT+CBC",8,32,5000,"OK"); //8,25
   Serial.print(F("---> Bateria: "));
   Serial.print(sensorValue);
   Serial.println();

//   itoa(sensorValue, CharBatt, 10);
//   dtostrf(sensorValue, 3, 0, CharBatt);  //mininum width, precision; first string copied into last array
//   sprintf(CharBatt, "%3d", sensorValue);

    itoa(sensorValue, CharBatt, 10);

//    memset(str_temp, '\0', sizeof(str_temp));
//    dtostrf(sensorValue, 3, 0, str_temp);
//    sprintf(CharBatt,"%s", str_temp);

    Serial.print("Char Batt: "); Serial.println(CharBatt);

   Serial.print(F("Time: "));
   Serial.println(millis() - previoustest);

   ////////////////////PM 2.5/////////////////
   
   Serial.println(F("---------------------------------------"));
   Serial.println(F("Particles PM 2.5!!!"));
   Serial.println(F("---------------------------------------"));
   
   SerialPM.listen();
   sensorValue=mide25(3,2); 

   Serial.print(F("---> Medida de Particulas PM 10: "));
   Serial.print(ValuePm10);
   Serial.print(F(" ug/m3"));
   Serial.println();

   memset(CharPm10, '\0', sizeof(CharPm10));
   //CharPm10=(char)ValuePm10;
   itoa(ValuePm10, CharPm10, 10);
   // dtostrf(ValuePm10, 4, 0, CharPm10); 
   // sprintf(CharPm25, "%5d", sensorValue);
   Serial.print("Char PM10: "); Serial.println(CharPm10); 

   Serial.print(F("---> Medida de Particulas PM 2.5: "));
   Serial.print(sensorValue);
   Serial.print(F(" ug/m3"));
   Serial.println();

   memset(CharPm25, '\0', sizeof(CharPm25));
   itoa(sensorValue, CharPm25, 10);
   // dtostrf(sensorValue, 4, 0, CharPm25);  //mininum width, precision; first string copied into last array
   // sprintf(CharPm25, "%5d", sensorValue);
   Serial.print("Char PM25: "); Serial.println(CharPm25); 
  
   SerialSIM.listen();
 
   Serial.print(F("Time: "));
   Serial.println(millis() - previoustest);
   
    ////////////////////CO value ////////////// 

    Serial.println(F("---------------------------------------"));
    Serial.println(F("Carbone Monoxide Level!!!"));
    Serial.println(F("---------------------------------------"));

    sensorValue=MeasureCO(2);

    Serial.print(F("---> CO: "));
    Serial.print(sensorValue);
    Serial.println();

    memset(CharCO, '\0', sizeof(CharCO));
    // dtostrf(sensorValue, 5, 0, CharCO );  //mininum width, precision; first string copied into last array
     itoa(sensorValue, CharCO, 10);
    // sprintf(CharCO, "%5d", sensorValue);
    Serial.print("Char CO: "); Serial.println(CharCO);

    Serial.print(F("Time: "));
    Serial.println(millis() - previoustest);
////////////////////O3 value ////////////// 

    Serial.println(F("---------------------------------------"));
    Serial.println(F("Ozone Level!!!"));
    Serial.println(F("---------------------------------------"));

   sensorValue=MeasureO3(2);

   Serial.print(F("---> O3: "));
   Serial.print(sensorValue);
   Serial.println();

   memset(CharO3, '\0', sizeof(CharO3));
   // dtostrf(sensorValue, 3, 0, CharO3);  //mininum width, precision; first string copied into last array
    itoa(sensorValue, CharO3, 10);
   // sprintf(CharO3, "%5d", sensorValue);
   Serial.print("Char O3: "); Serial.println(CharO3);

  Serial.print(F("Time: "));
  Serial.println(millis() - previoustest);
   
  ////////////////////Humidity & Temp/////////////

    if(digitalRead(DCSupply)){BlinkLeds(4);}  
        
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Humidity & Temperature!!!"));
    Serial.println(F("---------------------------------------"));  

    float fh = dht.readHumidity();
    float ft = dht.readTemperature();


    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(ft) || isnan(fh)) 
    {
        Serial.println();
        Serial.println(F("Failed to read from DHT"));
    } 
    else 
    {
        memset(CharT, '\0', sizeof(CharT));
        memset(CharH, '\0', sizeof(CharH));
        
        memset(str_temp, '\0', sizeof(str_temp));
        Serial.print(F("---> Humidity: ")); 
        Serial.print(fh);
        Serial.println(F(" %\t"));
        
        dtostrf(fh, 4, 2, str_temp);
        sprintf(CharH,"%s", str_temp);

        memset(str_temp, '\0', sizeof(str_temp));
        Serial.print(F("---> Temperature: ")); 
        Serial.print(ft);
        Serial.println(F(" *C"));

        dtostrf(ft, 4, 2, str_temp);
        sprintf(CharT,"%s", str_temp);        
        // sprintf(CharT, "%4.2f", ft);               
    }
   Serial.print("Char T: "); Serial.println(CharT);
   Serial.print("Char H: "); Serial.println(CharH);

  Serial.print(F("Time: "));
  Serial.println(millis() - previoustest);
  
  ////////////////////GPS String //////////////
  
    Serial.println(F("---------------------------------------"));
    Serial.println(F("GPS"));
    Serial.println(F("---------------------------------------"));
   //Serial.println(F("Determining GPS"));
   
  
    memset(GPStext1,'\0', sizeof(GPStext1));
    memset(GPStext2,'\0', sizeof(GPStext2));
    memset(GPStext3,'\0', sizeof(GPStext3));

    GPSFunction();GPSFunction();
    Serial.print("GPS Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" - Quality: "); Serial.println((int)GPS.fixquality); 
    if (((int)GPS.fix==1)&&((int)GPS.fixquality>=2)) { 
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
              if (testmode){
                dtostrf(GPS.latitudeDegrees+dummyGPS+0.005, 10, 6, GPStext1); 
                dtostrf(GPS.longitudeDegrees+dummyGPS, 10, 6, GPStext2);
                dummyGPS=dummyGPS+0.00001; 
              }else{
                dtostrf(GPS.latitudeDegrees, 10, 6, GPStext1); 
                dtostrf(GPS.longitudeDegrees, 10, 6, GPStext2);
              }
      
      Serial.print("Lat (string): ");
      Serial.print(GPStext1);
      Serial.println();
      Serial.print("Lon (string): ");
      Serial.print(GPStext2);
      Serial.println();     

      Serial.print("Difference: ");
      Serial.print(oldGPSlat-GPS.latitudeDegrees,6);
      Serial.print(","); 
      Serial.println(oldGPSlon-GPS.longitudeDegrees,6);
      
      oldGPSlat=GPS.latitudeDegrees;
      oldGPSlon=GPS.longitudeDegrees;

//     memset(str_temp2, '\0', sizeof(str_temp2));
//     dtostrf(GPStext1, 9, 6, str_temp2);
//     sprintf(GPS,"%s", str_temp2);
  
      snprintf(GPStext3, sizeof(GPStext3), "%s,%s",GPStext1,GPStext2 );
      
      Serial.print("---> GPS (string): ");
      Serial.print(GPStext3);
      Serial.println();
           
    }

   Serial.print(F("Time: "));
   Serial.println(millis() - previoustest);
  
  ////////////////////////////////// 
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Sending text to the internet"));
  Serial.println(F("---------------------------------------"));
  
  Serial.print("Char PM25: "); Serial.println(CharPm25); 
  Serial.print("Char PM10: "); Serial.println(CharPm10); 
  Serial.print("Char O3: "); Serial.println(CharO3); 
  Serial.print("Char CO: "); Serial.println(CharCO);
  Serial.print("Char Batt: "); Serial.println(CharBatt); 
  Serial.print("Char T: "); Serial.println(CharT); 
  Serial.print("Char H: "); Serial.println(CharH); 
  Serial.print("Char GPS 1: "); Serial.println(GPStext1); 
  Serial.print("Char GPS 2: "); Serial.println(GPStext2); 
  
  SerialSIM.listen();

  //if (digitalRead(SwitchON)){GETSendLocation2(CharPm25,CharPm10,CharO3,CharCO,CharBatt,CharT,CharH,GPStext1,GPStext2);}
  if (digitalRead(SwitchON)){GETSendLocation3(CharPm25,CharPm10,CharO3,CharCO,CharBatt,CharT,CharH,GPStext3);}
  Serial.print(F("Time: "));
  Serial.println(millis() - previoustest);

  ////////////////////////////////// 
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Checking Parameters"));
  Serial.println(F("---------------------------------------"));
   
  Serial.print(F("Battery Voltage: ")); 
  Serial.print(BattVolt);
  Serial.println(F(" V"));

  Serial.print(F("DC Supply Connected: "));
  Serial.println(digitalRead(DCSupply));

  Serial.print(F("GPS ON: "));
  Serial.println(digitalRead(GPSInput));

  Serial.print(F("Switch ON: "));
  Serial.println(digitalRead(SwitchON));

  Serial.print(F("Time: "));
  Serial.println(millis() - previoustest);
  
  Serial.println(F("Awaiting some seconds for next measurement..."));
  Serial.println(F("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooo"));

    ////////////////////End of Concatenating ALL/////////////
 }
  
}else{
 
  ////////////////////INDOOR routine ////////////// 
  if ((currentMillis - previousMillis2 >= 60000*5 && InternetEnabled)||(StartLecture && InternetEnabled)) {
    
    previoustest = millis();

    if(!StartLecture){previousMillis2 = currentMillis;} 
    StartLecture=false;  
  
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Indoors routine: every 5 minutes!"));
    Serial.println(F("---------------------------------------"));

 ////////////////////Battery value ////////////// 
 for (int m=1;m<=3;m++)
 {
   Serial.println(F("---------------------------------------"));
   Serial.print(F("Loop value = "));
   Serial.println(m);  
   
   if(m!=1){
          if(!digitalRead(DCSupply)){delay(3000);}
          if(digitalRead(DCSupply)){delay(2000);BlinkLeds(4);}
          if(!digitalRead(DCSupply)){delay(3000);}
          if(digitalRead(DCSupply)){delay(2000);BlinkLeds(4);}
          if(!digitalRead(DCSupply)){delay(3000);}
          if(digitalRead(DCSupply)){delay(2000);BlinkLeds(4);}
          if(!digitalRead(DCSupply)){delay(3000);}
          if(digitalRead(DCSupply)){delay(2000);BlinkLeds(4);}
          if(!digitalRead(DCSupply)){delay(3000);}
          if(digitalRead(DCSupply)){delay(2000);BlinkLeds(4);}
    } // time between measurements
            
   Serial.println(F("---------------------------------------"));
   
   Serial.println(F("---------------------------------------"));
   Serial.println(F("Battery Level!!!"));
   Serial.println(F("---------------------------------------"));
   SerialSIM.listen();
   sensorValue=0;

   sendATcommand("AT+CBC", "OK", 5000);
   sendATcommand("AT+CBC", "OK", 5000);
   sensorValue=RetrieveBatt("AT+CBC",8,32,5000,"OK"); //8,25
   Serial.print(F("---> Bateria: "));
   Serial.print(sensorValue);
   Serial.println();
   bufferBatt[m]=sensorValue;

//   itoa(sensorValue, CharBatt, 10);
//   dtostrf(sensorValue, 3, 0, CharBatt);  //mininum width, precision; first string copied into last array
//   sprintf(CharBatt, "%3d", sensorValue);

//    memset(str_temp, '\0', sizeof(str_temp));
//    dtostrf(sensorValue, 3, 0, str_temp);
//    sprintf(CharBatt,"%s", str_temp);

   ////////////////////PM 2.5/////////////////
   
   Serial.println(F("---------------------------------------"));
   Serial.println(F("Particles PM 2.5!!!"));
   Serial.println(F("---------------------------------------"));
   
   SerialPM.listen();
   sensorValue=mide25(3,8); 

   Serial.print(F("---> Medida de Particulas PM 10: "));
   bufferPM10[m]=ValuePm10;
   Serial.print(bufferPM10[m]);
   Serial.print(F(" ug/m3"));
   Serial.println();

   Serial.print(F("---> Medida de Particulas PM 2.5: "));
   Serial.print(sensorValue);
   Serial.print(F(" ug/m3"));
   Serial.println();
   bufferPM25[m]=sensorValue;
  
   SerialSIM.listen();
   
   if(digitalRead(DCSupply)){BlinkLeds(4);}   
    ////////////////////CO value ////////////// 

    Serial.println(F("---------------------------------------"));
    Serial.println(F("Carbone Monoxide Level!!!"));
    Serial.println(F("---------------------------------------"));

    sensorValue=MeasureCO(2);

    Serial.print(F("---> CO: "));
    Serial.print(sensorValue);
    Serial.println();
    bufferCO[m]=sensorValue;
    
////////////////////O3 value ////////////// 

    Serial.println(F("---------------------------------------"));
    Serial.println(F("Ozone Level!!!"));
    Serial.println(F("---------------------------------------"));

   sensorValue=MeasureO3(2);

   Serial.print(F("---> O3: "));
   Serial.print(sensorValue);
   Serial.println();
   bufferO3[m]=sensorValue;
   
  if(digitalRead(DCSupply)){BlinkLeds(4);}   
  ////////////////////Humidity & Temp/////////////
        
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Humidity & Temperature!!!"));
    Serial.println(F("---------------------------------------"));  

    float fh = dht.readHumidity();
    float ft = dht.readTemperature();


    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(ft) || isnan(fh)) 
    {
        Serial.println();
        Serial.println(F("Failed to read from DHT"));
    } 
    else 
    {                
        Serial.print(F("---> Humidity: ")); 
        bufferH[m]=fh;
        Serial.print(bufferH[m]);
        Serial.println(F(" %\t"));
        
        Serial.print(F("---> Temperature: ")); 
        bufferT[m]=ft;
        Serial.print(bufferT[m]);
        Serial.println(F(" *C"));            
    }

 if(digitalRead(DCSupply)){BlinkLeds(4);}  
 }
 
  
  ////////////////////////////////// making the average
 
       Serial.println(F("---------- "));
       Serial.print(F("PM 2.5 recolectados: "));
       Serial.print(bufferPM25[1]);
       Serial.print(F(","));
       Serial.print(bufferPM25[2]);
       Serial.print(F(","));
       Serial.print(bufferPM25[3]);
       Serial.print(F(" - PM 2.5 average: "));
       answer1=round((bufferPM25[1]+bufferPM25[2]+bufferPM25[3])/3);
       Serial.println(answer1);

       memset(CharPm25, '\0', sizeof(CharPm25));
       itoa(answer1, CharPm25, 10);
       
       Serial.println(F("---------- "));
       Serial.print(F("PM 10 recolectados: "));
       Serial.print(bufferPM10[1]);
       Serial.print(F(","));
       Serial.print(bufferPM10[2]);
       Serial.print(F(","));
       Serial.print(bufferPM10[3]);
       Serial.print(F(" - PM 10 average: "));
       answer1=round((bufferPM10[1]+bufferPM10[2]+bufferPM10[3])/3);
       Serial.println(answer1);

       memset(CharPm10, '\0', sizeof(CharPm10));
       itoa(answer1, CharPm10, 10);

       Serial.println(F("---------- "));
       Serial.print(F("Batt recolectados: "));
       Serial.print(bufferBatt[1]);
       Serial.print(F(","));
       Serial.print(bufferBatt[2]);
       Serial.print(F(","));
       Serial.print(bufferBatt[3]);
       Serial.print(F(" - Batt average: "));
       answer1=round((bufferBatt[1]+bufferBatt[2]+bufferBatt[3])/3);
       Serial.println(answer1);

       memset(CharBatt, '\0', sizeof(CharBatt));
       itoa(answer1, CharBatt, 10);

       Serial.println(F("---------- "));
       Serial.print(F("CO recolectados: "));
       Serial.print(bufferCO[1]);
       Serial.print(F(","));
       Serial.print(bufferCO[2]);
       Serial.print(F(","));
       Serial.print(bufferCO[3]);
       Serial.print(F(" - CO average: "));
       answer1=round((bufferCO[1]+bufferCO[2]+bufferCO[3])/3);
       Serial.println(answer1);

       memset(CharCO, '\0', sizeof(CharCO));
       //answer1=1221;
       itoa(answer1, CharCO, 10);

       Serial.println(F("---------- "));
       Serial.print(F("O3 recolectados: "));
       Serial.print(bufferO3[1]);
       Serial.print(F(","));
       Serial.print(bufferO3[2]);
       Serial.print(F(","));
       Serial.print(bufferO3[3]);
       Serial.print(F(" - O3 average: "));
       answer1=round((bufferO3[1]+bufferO3[2]+bufferO3[3])/3);
       Serial.println(answer1);

       memset(CharO3, '\0', sizeof(CharO3));
       //answer1=1234;
       itoa(answer1, CharO3, 10);
       
       Serial.println(F("---------- "));
       Serial.print(F("Temp recolectados: "));
       Serial.print(bufferT[1]);
       Serial.print(F(","));
       Serial.print(bufferT[2]);
       Serial.print(F(","));
       Serial.print(bufferT[3]);
       Serial.print(F(" - Temp average: "));
       answerf=((bufferT[1]+bufferT[2]+bufferT[3])/3);
       Serial.println(answerf);

       memset(CharT, '\0', sizeof(CharT));
       memset(str_temp, '\0', sizeof(str_temp));
       dtostrf(answerf, 4, 2, str_temp);
       sprintf(CharT,"%s", str_temp);
       
       Serial.println(F("---------- "));
       Serial.print(F("Hum recolectados: "));
       Serial.print(bufferH[1]);
       Serial.print(F(","));
       Serial.print(bufferH[2]);
       Serial.print(F(","));
       Serial.print(bufferH[3]);
       Serial.print(F(" - Hum average: "));
       answerf=((bufferH[1]+bufferH[2]+bufferH[3])/3);
       Serial.println(answerf);

       memset(CharH, '\0', sizeof(CharH));
       memset(str_temp, '\0', sizeof(str_temp));
       dtostrf(answerf, 4, 2, str_temp);
       sprintf(CharH,"%s", str_temp);

  ////////////////////////////////// 
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Sending text to the internet"));
  Serial.println(F("---------------------------------------"));
  
  Serial.print("Char PM25: "); Serial.println(CharPm25); 
  Serial.print("Char PM10: "); Serial.println(CharPm10); 
  Serial.print("Char O3: "); Serial.println(CharO3); 
  Serial.print("Char CO: "); Serial.println(CharCO);
  Serial.print("Char Batt: "); Serial.println(CharBatt); 
  Serial.print("Char T: "); Serial.println(CharT); 
  Serial.print("Char H: "); Serial.println(CharH); 

  if(digitalRead(DCSupply)){BlinkLeds(4);}  
  
  SerialSIM.listen();
  if (digitalRead(SwitchON)){GETSend2(CharPm25,CharPm10,CharO3,CharCO,CharBatt,CharT,CharH);}
  if(digitalRead(DCSupply)){BlinkLeds(4);}   
  ////////////////////////////////// 
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Checking Parameters"));
  Serial.println(F("---------------------------------------"));
   
  Serial.print(F("Battery Voltage: ")); 
  Serial.print(BattVolt);
  Serial.println(F(" V"));

  Serial.print(F("DC Supply Connected: "));
  Serial.println(digitalRead(DCSupply));

  Serial.print(F("GPS ON: "));
  Serial.println(digitalRead(GPSInput));

  Serial.print(F("Switch ON: "));
  Serial.println(digitalRead(SwitchON));

  Serial.print(F("Time: "));
  Serial.println(millis() - previoustest);
  
  Serial.println(F("Awaiting some minutes for next measurement..."));
  Serial.println(F("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooo"));
   
 }
  
}
 

}


void close_3G(){
  //Serial.println(F("Closing Connection..."));
  sendATcommand("AT+CLTS=0", "OK", 2000); // "Get Local Time" Stamp Disabled 
  sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
  sendATcommand("AT+HTTPTERM", "OK", 4000); // closing http if open
  // Clean Assigned IP
}

boolean power_on(){
  boolean turnedon=false;
  BlinkLeds(1);BlinkLeds(1); //All leds
  Serial.println(F("Turning SIM900 ON...")); 
  delay(50);
  if (sendATcommand("AT+CGMM=?", "OK", 100) == 1)
    {// Nothing, the SIM900 is ON
    Serial.println(F("SIM Already ON!"));
    }
    else
    {  
      Serial.println(F("Turning SIM Now!"));
      digitalWrite(SIMPower,HIGH);
      delay(1400);
      digitalWrite(SIMPower,LOW);
    }

  Serial.println(F("Code to start connection!"));
  sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications
  
  countloop=0;    
  while( (sendATcommand("AT+CREG?", "+CREG: 1,1", 600) // before 0,1
            || sendATcommand("AT+CREG?", "+CREG: 0,5", 600)
            || sendATcommand("AT+CREG?", "+CREG: 0,1", 600)
            || sendATcommand("AT+CREG?", "+CREG: 2,1", 600)) == 0 )
            {countloop++; BlinkLeds(2); if(countloop >= 8){break;}};  //YYYYY If it does not find network think what to do
            
  sendATcommand("AT+CREG=0", "OK", 5000); // Deactivating CREG notifications
  BlinkLeds(2); // Data led
  //Serial.println(F("Connected to the network!"));
  if(countloop < 8){turnedon=true; Serial.println(F("Connected to the network!"));}
  else {Serial.println(F("NOT Connected to the network!"));}
  return turnedon;
}

void power_off(){
   Serial.println(F("Turning SIM900 OFF!")); 
   countloop=0;    
   while (sendATcommand("AT+CPOWD=1", "DOWN", 5000) != 1) 
        {countloop++; if(countloop >= 3){break;}}; 
}

//int BatteryLevel(){
//  char StringResult[40];   // BEFORE IT WAS 15 o 60
//  char StringResult2[3];
//  char* pch;
//  memset(StringResult, '\0', 40);
//  memset(StringResult2, '\0', 3);
// 
//  uint8_t StartIndex=11;//16
//  uint8_t EndIndex=25; //26
//  
//  sendATcommand("AT+CBC", "OK", 5000);
//  
//  memcpy(StringResult,RetrieveATResponse("AT+CBC",StartIndex,EndIndex,15000,"OK"),EndIndex-StartIndex);
//  
//  Serial.println(F("First String:"));
//  Serial.println(StringResult);
// 
//  pch= strstr(StringResult, "CBC:");
//  memcpy(StringResult2,pch+7,3);
//  
//  Serial.println(F("Second String:"));
//  Serial.println(StringResult2);
//  
//  int x=atoi(StringResult2);
//  
//  Serial.println(F("Third String: "));
//  Serial.println(x);
//    
//  return x; 
//  
//}

boolean configure_3G(){
    boolean InternetOn=false;
    char aux_str[45]; 
    
    Serial.println(F("Configuring 3G Connection...")); 
 
    sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
    
    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
    sendATcommand(aux_str, "OK", 2000);
    
    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
    sendATcommand(aux_str, "OK", 2000);
    BlinkLeds(2); //Data led
    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
    sendATcommand(aux_str, "OK", 2000);
    BlinkLeds(2); //Data led

    countloop=0;
    while ( (  sendATcommand("AT+SAPBR=1,1", "OK", 6000)
            || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 6000) ) != 1)
           {countloop++;BlinkLeds(2);if(countloop >= 10){break;}}; 
  
  if(countloop < 5){InternetOn=true; Serial.println(F("Connected to the Internet!"));}
  else {Serial.println(F("NOT Connected to the Internet!"));}
         
    /* 
    int StartIndex=31;
    int EndIndex=46;
    memcpy(AssignedIP, RetrieveATResponse("AT+SAPBR=2,1",StartIndex,EndIndex,8000),EndIndex-StartIndex);
    Serial.println("Assigned IP:");
    Serial.write(AssignedIP,EndIndex-StartIndex);
    Serial.println();
    */    
  
     countloop=0;    
     while (sendATcommand("AT+HTTPINIT", "OK", 1000) != 1)  
           {countloop++; if(countloop >= 3){break;}};
     delay(100);       
     sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 10000); 
     
     return InternetOn;
}


void GETSendLocation3(char* VCharPm25,char* VCharPm10,char* VCharO3,char* VCharCO,char* VCharBatt,char* VCharT,char* VCharH, char* VCharLoc3){
    
    char aux_str[370];
    memset(aux_str, '\0', 370);

    int countlimit2g= 50; // # data upload tries
    int evillimit1=10; // limit reset 2g
    int evillimit2=20; // limit turn off SIM

       snprintf(aux_str, sizeof(aux_str),"AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL%sdeviceID=0002%sbatt=%s%sgps=%s%stemp=%s%shr=%s%spm25=%s%spm10=%s%so3=%s%sco=%s%smode=1\"","?","&",VCharBatt,"&",VCharLoc3,"&",VCharT,"&",VCharH,"&",VCharPm25,"&",VCharPm10,"&",VCharO3,"&",VCharCO,"&");       
       
       if (DevelopmentMode == true){Serial.println("ATcommand: ");Serial.println(aux_str);}
       
       Serial.println("Enviar a la internet!: ");
       sendATcommand(aux_str, "OK", 1000);

     countloop=1;                                             
     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 11000) != 1) //45 is the lenght of char text from file csv2sql
           {delay(3);if(countloop >= countlimit2g){break;};countloop++;};

     if((countloop >=countlimit2g) && (evilcounter < evillimit1)){
         Serial.print(F("Small mistake - No Reset - Just wait - "));
         Serial.print(F("EC: "));
         Serial.println(evilcounter);
         
         evilcounter=evilcounter+1;
       }
       
       if((countloop >=countlimit2g) && (evilcounter >= evillimit1) && (evilcounter < evillimit2) ){
         Serial.print(F("Small mistake - SEVERAL TIMES - 2G Reset - "));
         Serial.print(F("EC: "));
         Serial.println(evilcounter);
         
         InternetEnabled=0;
         digitalWrite(DataLed,InternetEnabled);
         close_3G(); 
         delay(20);
         InternetEnabled=configure_3G();
         digitalWrite(DataLed,InternetEnabled);

         evilcounter=evilcounter+1;
       }
           
        if((countloop >=countlimit2g) && (evilcounter >= evillimit2)){
           Serial.print(F("BIG mistake - SIM OFF - "));
           Serial.print(F("EC: "));
           Serial.println(evilcounter);

           InternetEnabled=0;
           digitalWrite(DataLed,InternetEnabled);
           close_3G();
           delay(800); 
           power_off(); 
           delay(800);
           power_on();
           delay(800);
           InternetEnabled=configure_3G();           

           int ii=0;
           while(!InternetEnabled){
             Serial.println(F("HUGE mistake - SIM OFF - "));
             Serial.print(F("EC: "));
             Serial.println(evilcounter);
             close_3G();
             delay(800); 
             power_off(); 
             delay(800);
             power_on(); 
             delay(800);
             InternetEnabled=configure_3G();
             ii=ii+1;
             if(ii >= 5){break;};
          } 
          digitalWrite(DataLed,InternetEnabled);

          evilcounter=evilcounter+1;

        }

     if(evilcounter>10000){evilcounter=0;};
 
     if(sendATcommand("AT+HTTPREAD", "k!", 5000)) // "k!"
     {
       Serial.println(F("Success in uploading data"));
       evilcounter=0;
       for(int j=0;j<8;j++)
       {
          digitalWrite(DataLed, !digitalRead(DataLed));
          delay(80);
       }       
     }

}


//void GETSendLocation2(char* VCharPm25,char* VCharPm10,char* VCharO3,char* VCharCO,char* VCharBatt,char* VCharT,char* VCharH, char* VCharLoc1, char* VCharLoc2){
//    
//    char aux_str[360];
//    memset(aux_str, '\0', 360);
//
//    int countlimit2g= 50; // # data upload tries
//    int evillimit1=10; // limit reset 2g
//    int evillimit2=20; // limit turn off SIM
//
//       snprintf(aux_str, sizeof(aux_str),"AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL%sdeviceID=0002%sbatt=%s%sgps=%s,%s%stemp=%s%shr=%s%spm25=%s%spm10=%s%so3=%s%sco=%s%smode=1\"","?","&",VCharBatt,"&",VCharLoc1,VCharLoc2,"&",VCharT,"&",VCharH,"&",VCharPm25,"&",VCharPm10,"&",VCharO3,"&",VCharCO,"&");       
//       
//       if (DevelopmentMode == true){Serial.println("ATcommand: ");Serial.println(aux_str);}
//       
//       Serial.println("Enviar a la internet!: ");
//       sendATcommand(aux_str, "OK", 1000);
//
//     countloop=1;                                             
//     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 11000) != 1) //45 is the lenght of char text from file csv2sql
//           {delay(3);if(countloop >= countlimit2g){break;};countloop++;};
//
//     if((countloop >=countlimit2g) && (evilcounter < evillimit1)){
//         Serial.print(F("Small mistake - No Reset - Just wait - "));
//         Serial.print(F("EC: "));
//         Serial.println(evilcounter);
//         
//         evilcounter=evilcounter+1;
//       }
//       
//       if((countloop >=countlimit2g) && (evilcounter >= evillimit1) && (evilcounter < evillimit2) ){
//         Serial.print(F("Small mistake - SEVERAL TIMES - 2G Reset - "));
//         Serial.print(F("EC: "));
//         Serial.println(evilcounter);
//         
//         InternetEnabled=0;
//         digitalWrite(DataLed,InternetEnabled);
//         close_3G(); 
//         delay(20);
//         InternetEnabled=configure_3G();
//         digitalWrite(DataLed,InternetEnabled);
//
//         evilcounter=evilcounter+1;
//       }
//           
//        if((countloop >=countlimit2g) && (evilcounter >= evillimit2)){
//           Serial.print(F("BIG mistake - SIM OFF - "));
//           Serial.print(F("EC: "));
//           Serial.println(evilcounter);
//
//           InternetEnabled=0;
//           digitalWrite(DataLed,InternetEnabled);
//           close_3G();
//           delay(800); 
//           power_off(); 
//           delay(800);
//           power_on();
//           delay(800);
//           InternetEnabled=configure_3G();           
//
//           int ii=0;
//           while(!InternetEnabled){
//             Serial.println(F("HUGE mistake - SIM OFF - "));
//             Serial.print(F("EC: "));
//             Serial.println(evilcounter);
//             close_3G();
//             delay(800); 
//             power_off(); 
//             delay(800);
//             power_on(); 
//             delay(800);
//             InternetEnabled=configure_3G();
//             ii=ii+1;
//             if(ii >= 5){break;};
//          } 
//          digitalWrite(DataLed,InternetEnabled);
//
//          evilcounter=evilcounter+1;
//
//        }
//
//     if(evilcounter>10000){evilcounter=0;};
// 
//     if(sendATcommand("AT+HTTPREAD", "k!", 5000)) // "k!"
//     {
//       Serial.println(F("Success in uploading data"));
//       evilcounter=0;
//       for(int j=0;j<8;j++)
//       {
//          digitalWrite(DataLed, !digitalRead(DataLed));
//          delay(80);
//       }       
//     }
//     
//
//     
//  
//}


//   GETSend2(CharPm25,CharPm10,CharO3,CharCO,CharBatt,CharT,CharH);
void GETSend2(char* VCharPm25,char* VCharPm10,char* VCharO3,char* VCharCO,char* VCharBatt,char* VCharT,char* VCharH){
    
    char aux_str[360];
    memset(aux_str, '\0', 360);

    int countlimit2g= 50; // # data upload tries
    int evillimit1=10; // limit reset 2g
    int evillimit2=20; // limit turn off SIM

       //Buena
       snprintf(aux_str, sizeof(aux_str),"AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL%sdeviceID=0002%sbatt=%s%stemp=%s%shr=%s%spm25=%s%spm10=%s%so3=%s%sco=%s%smode=0\"","?","&",VCharBatt,"&",VCharT,"&",VCharH,"&",VCharPm25,"&",VCharPm10,"&",VCharO3,"&",VCharCO,"&");       
      
       // bypassear
       // snprintf(GPStext3, sizeof(GPStext3), "%s,%s",GPStext1,GPStext2 );
       // snprintf(aux_str, sizeof(aux_str),"AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL%sdeviceID=0002%sbatt=%s%sgps=%s%stemp=%s%shr=%s%spm25=%s%spm10=%s%so3=%s%sco=%s%smode=1\"","?","&",VCharBatt,"&",GPStext3,"&",VCharT,"&",VCharH,"&",VCharPm25,"&",VCharPm10,"&",VCharO3,"&",VCharCO,"&");       
       
       if (DevelopmentMode == true){Serial.println("ATcommand: ");Serial.println(aux_str);}

       Serial.println("Enviar a la internet!: ");
       sendATcommand(aux_str, "OK", 1000);

     countloop=1;                                              //26 for other
     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 11000) != 1) //45 is the lenght of char text from file csv2sql
           {delay(3);if(countloop >= countlimit2g){break;};countloop++;}; 
     
       if((countloop >=countlimit2g) && (evilcounter < evillimit1)){
         Serial.print(F("Small mistake - No Reset - Just wait - "));
         Serial.print(F("EC: "));
         Serial.println(evilcounter);
         
         evilcounter=evilcounter+1;
       }
       
       if((countloop >=countlimit2g) && (evilcounter >= evillimit1) && (evilcounter < evillimit2) ){
         Serial.print(F("Small mistake - SEVERAL TIMES - 2G Reset - "));
         Serial.print(F("EC: "));
         Serial.println(evilcounter);
         
         InternetEnabled=0;
         digitalWrite(DataLed,InternetEnabled);
         close_3G(); 
         delay(20);
         InternetEnabled=configure_3G();
         digitalWrite(DataLed,InternetEnabled);

         evilcounter=evilcounter+1;
       }
           
        if((countloop >=countlimit2g) && (evilcounter >= evillimit2)){
           Serial.print(F("BIG mistake - SIM OFF - "));
           Serial.print(F("EC: "));
           Serial.println(evilcounter);

           InternetEnabled=0;
           digitalWrite(DataLed,InternetEnabled);
           close_3G();
           delay(800); 
           power_off(); 
           delay(800);
           power_on();
           delay(800);
           InternetEnabled=configure_3G();           

           int ii=0;
           while(!InternetEnabled){
             Serial.println(F("HUGE mistake - SIM OFF - "));
             Serial.print(F("EC: "));
             Serial.println(evilcounter);
             close_3G();
             delay(800); 
             power_off(); 
             delay(800);
             power_on(); 
             delay(800);
             InternetEnabled=configure_3G();
             ii=ii+1;
             if(ii >= 5){break;};
          } 
          digitalWrite(DataLed,InternetEnabled);

          evilcounter=evilcounter+1;

        }     

     if(evilcounter>10000){evilcounter=0;};
 
     if(sendATcommand("AT+HTTPREAD", "k!", 5000)) // "k!"
     {
       Serial.println(F("Success in uploading data"));
       evilcounter=0;
       for(int j=0;j<8;j++)
       {
          digitalWrite(DataLed, !digitalRead(DataLed));
          delay(80);
       }       
     }
                               
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[500]; //270
    unsigned long previous;

    memset(response, '\0', 500);    // Initialize the string

    //delay(10);

    while( SerialSIM.available() > 0) SerialSIM.read();    // Clean the input buffer

    SerialSIM.println(ATcommand);    // Send the AT command

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(SerialSIM.available() != 0){ 
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = SerialSIM.read();
            //delay(5); 
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
//            if (strstr(response, "+HTTPACTION: 0,200,132") != NULL)    
//            {
//              Serial.println(F("HTTP Error - New Platform Text"));
//                answer = 2;                
//            }
            if (strstr(response, "Error") != NULL)    
            {
              Serial.println(F("HTTP Error - New Platform Text 2"));
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

    if (DevelopmentMode == true){Serial.println(response);} // Send the AT command
         
    if((millis() - previous) > timeout){
        Serial.print(F("Time exceeded: "));
        Serial.println(millis() - previous);
    }  
    
    return answer;
}
/*
int8_t sendATcommandDebug(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[180];
    unsigned long previous;

    memset(response, '\0', 180);    // Initialize the string

    delay(50);

    while( SerialSIM.available() > 0) SerialSIM.read();    // Clean the input buffer

    SerialSIM.println(ATcommand);    // Send the AT command

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(SerialSIM.available() != 0){ 
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = SerialSIM.read();
            
            Serial.print("|R:");
            Serial.print(response);
            Serial.print("|");
            //delay(2);
            
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL)    
            {
               //Serial.print("Match!");  
               answer = 1;
            }
            if (strstr(response, "+FTPPUT:1,6") != NULL)    
            {
              Serial.println(F("FTP Error Code"));
                answer = 2;                
            }
        }
    }
    // Waits for the asnwer with time out    
    while((answer == 0) && ((millis() - previous) < timeout));
    
    if((millis() - previous) > timeout){
        Serial.print(F("Time exceeded: "));
        Serial.println(millis() - previous);
    }    
    
    Serial.println(response);    // Send the AT command
    return answer;
}
*/
char* RetrieveATResponse(char* ATcommand, unsigned int InitialIndex, unsigned int FinalIndex, unsigned int timeout, char* expected_answer) //22,39,1000
 {
  
  char buffer1[180]; // buffer array for data recieve over serial port //BEFORE IT WAS 50
  char buffer2[FinalIndex-InitialIndex]; // buffer array for data recieve over serial port
  uint8_t answer=0;
  
  memset(buffer1, '\0', 180);
  memset(buffer2, '\0', FinalIndex-InitialIndex);
  
  char StringResult[FinalIndex-InitialIndex];
  int k=0;
  int count = 0;
  unsigned long previoustemp;
  
  previoustemp = millis();  
  
  SerialSIM.println(ATcommand);       // write it to the GPRS shield
  Serial.println();
  do{
    if (SerialSIM.available())              // if date is comming from softwareserial port ==> data is comming from gprs shield
    {

       buffer1[count++]=SerialSIM.read();     // writing data into array
       if(count == 100)break; 
       
       if (strstr(buffer1, expected_answer) != NULL)    
            {
               //Serial.print("Match!");  
               answer = 1;
            }
       
     }
  }while((answer == 0) && ((millis() - previoustemp) < timeout));
    
  if((millis() - previoustemp) > timeout){
        Serial.print(F("Time exceeded: "));
        Serial.println(millis() - previoustemp);
    }
  
 k=0;
 for (int i=InitialIndex; i<FinalIndex;i++){ buffer2[k]=buffer1[i];k++;}  //saving to a variable
 buffer2[k]={'\n'};
 
 /*
 Serial.println("\n Buffer1 \n");
 Serial.write(buffer1,50);            // if no data transmission ends, write buffer to hardware serial port
 Serial.println("\n Buffer2 \n");
 Serial.write(buffer2,FinalIndex-InitialIndex);            // if no data transmission ends, write buffer to hardware serial port
 delay(200);
 */
 memcpy(StringResult, buffer2, sizeof(buffer2));
 
 count = 0;                       // set counter of while loop to zero
    
 return StringResult;
  
 }

void BlinkLeds(int ledcommand)
{ 
    //
    boolean templed1=digitalRead(DataLed);
    boolean templed2=digitalRead(GPSLed);
    boolean templed3=digitalRead(BattLed);
    int delayled = 250;
    
    switch (ledcommand) { 
      
    case 1:  
    digitalWrite(DataLed, LOW);
    digitalWrite(GPSLed, LOW);
    digitalWrite(BattLed, LOW);
    delay(delayled);
    digitalWrite(DataLed, HIGH);
    digitalWrite(GPSLed, HIGH);
    digitalWrite(BattLed, HIGH);
    delay(delayled);
    digitalWrite(DataLed, LOW);
    digitalWrite(GPSLed, LOW);
    digitalWrite(BattLed, LOW);
    delay(delayled);
    digitalWrite(DataLed, HIGH);
    digitalWrite(GPSLed, HIGH);
    digitalWrite(BattLed, HIGH);
    delay(delayled);
    break;   

    case 2: 
    digitalWrite(DataLed, LOW);
    delay(delayled);
    digitalWrite(DataLed, HIGH);
    delay(delayled);
    digitalWrite(DataLed, LOW);
    delay(delayled);
    digitalWrite(DataLed, HIGH);
    delay(delayled);
    break;   

    case 3: 
    digitalWrite(GPSLed, LOW);
    delay(delayled);
    digitalWrite(GPSLed, HIGH);
    delay(delayled);
    digitalWrite(GPSLed, LOW);
    delay(delayled);
    digitalWrite(GPSLed, HIGH);
    delay(delayled);
    break;   

    case 4: 
    digitalWrite(BattLed, LOW);
    delay(delayled);
    digitalWrite(BattLed, HIGH);
    delay(delayled);
    digitalWrite(BattLed, LOW);
    delay(delayled);
    digitalWrite(BattLed, HIGH);
    delay(delayled);
    break;   

    default: //Read
    Serial.println(F("Invalid case"));
  }   
    
    digitalWrite(DataLed, templed1);
    digitalWrite(GPSLed, templed2); 
    digitalWrite(BattLed, templed3);

}

boolean SearchGPS(){
boolean GPSFound=true;
int count = 0;
  GPSFunction();GPSFunction();
  while((int)GPS.fix!= 1)
    {
      GPSFunction();GPSFunction();
      Serial.print("GPS Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" - Quality: "); Serial.println((int)GPS.fixquality);
      count++;if(count == 25)break;   
    }
if (((int)GPS.fix==1)&&((int)GPS.fixquality>=2)){GPSFound=true;}else{GPSFound=false;};
Serial.print("******  Good GPS Found: "); Serial.println(GPSFound);
return GPSFound;
}

double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}




int RetrieveBatt(char* ATcommand, unsigned int InitialIndex, unsigned int FinalIndex, unsigned int timeout, char* expected_answer) //22,39,1000
 {
  
  char buffer1[28]; // buffer array for data recieve over serial port //BEFORE IT WAS 50
  char buffer2[FinalIndex-InitialIndex+1]; // buffer array for data recieve over serial port
  uint8_t answer=0;
  char StringResult2[3];//yy
  char* pch;
  
  memset(buffer1, '\0', 28);
  memset(buffer2, '\0', FinalIndex-InitialIndex+1);
  
  char StringResultT[sizeof(buffer2)];
  memset(StringResultT, '\0', sizeof(buffer2));
  int k=0;
  int count = 0;
  unsigned long previoustemp;
  
  previoustemp = millis();  
  
  SerialSIM.println(ATcommand);       // write it to the GPRS shield
  Serial.println();
  do{
    if (SerialSIM.available())              // if date is comming from softwareserial port ==> data is comming from gprs shield
    {

       buffer1[count++]=SerialSIM.read();     // writing data into array
       if(count == 40)break; 
       
       if (strstr(buffer1, expected_answer) != NULL)    
            {
               //Serial.println("Match!");  
               answer = 1;
            }
       
     }
  }while((answer == 0) && ((millis() - previoustemp) < timeout));
    
  if((millis() - previoustemp) > timeout){
        Serial.print(F("Time exceeded: "));
        Serial.println(millis() - previoustemp);
    }
  
 k=0;
 for (int i=InitialIndex; i<FinalIndex;i++){ buffer2[k]=buffer1[i];k++;}  //saving to a variable
 buffer2[k]={'\n'};

 strcpy(StringResultT,buffer2);

 Serial.println("String Retrived after: ");
 Serial.print(StringResultT);
 Serial.println();

  pch= strstr(StringResultT, "CBC:");
  memcpy(StringResult2,pch+7,3);
  
  Serial.println(F("Second String:"));
  Serial.println(StringResult2);
  
  int x=atoi(StringResult2);

  if (x>=100){x=99;}
  
  Serial.println(F("Third String: "));
  Serial.println(x);
  count = 0; 
  return x;                      
 }



int mide25(byte a, byte b) // case, iterations OR coeff
{
  
  //1 stop
  //2 start, post-iterations
  //3 read PM25, pre-iterations
  //4 read coeff
  //4 set coeff, new coeff
  
  const byte readstart[] = {0x68, 0x01, 0x01, 0x96}; //Listed command to start particle measurement
  const byte readpart[] = {0x68, 0x01, 0x04, 0x93};  //Listed command to read particle data
  
  const byte readstop[] = {0x68, 0x01, 0x02, 0x95}; //Listed command to stop particle measurement
  const byte disableauto[] = {0x68, 0x01, 0x20, 0x77}; //Stop auto send
  
  byte setcoeff[] = {0x68, 0x02, 0x08, 0x64, 0x2A}; //Stop auto send
  const byte readcoeff[] = {0x68, 0x01, 0x10, 0x87}; //Stop auto send
  
  unsigned int answer;
  byte buffer1[12];
  int count = 0;
  memset(buffer1, '\0', sizeof(buffer1));
  
  switch (a) {
  case 1:  //Stop    
    Serial.println(F("---------------------"));
    Serial.println(F("Disable & Stop PM25..."));
    SerialPM.write(readstop, 4);
    delay(500);  
  break;
  case 2:  //Start
    Serial.println(F("---------------------"));
    Serial.println(F("Start & Disable & Pre-stabilize PM25..."));
    SerialPM.write(readstart, 4);
    delay(500);
    SerialPM.write(disableauto, 4);
    for(int j=0;j<=b;j++) // Modify "number" to stabilize measure
      {
        Serial.print(j); 
        Serial.print(F(" - "));
        SerialPM.write(readpart, 4);
        while(SerialPM.available()){
          Serial.print(SerialPM.read(),HEX);  // Data received and printed in the Serial Monitor.
          Serial.print(F(","));
        }
        Serial.println();
        delay(250); // yyy play with this number
      }

    break;
  case 3:  //Read
      Serial.println(F("---------------------"));
      Serial.println(F("Stabilize & read PM25..."));
      for(int j=0;j<=b;j++) // Modify "number" to stabilize measure
      {
        Serial.print(j); 
        Serial.print(F(" - "));
        SerialPM.write(readpart, 4);
        while(SerialPM.available()){
          Serial.print(SerialPM.read(),HEX);  // Data received and printed in the Serial Monitor.
          Serial.print(F(","));
        }
        Serial.println();
        delay(170); // yyy play with this number 250
      }
     
      Serial.println();
      Serial.println(F("Measurement PM25"));
      SerialPM.write(readpart, 4);
      delay (70); // yyy play with this number 50
      do{
        buffer1[count++]=SerialPM.read();     // writing data into array   
           if(count == 12)break;
       }while(SerialPM.available() );
       
     Serial.println(); 
     Serial.print(F("Buffer: "));
     for(int i=0;i<sizeof(buffer1);i++)
     {
     Serial.print(buffer1[i],HEX);
     Serial.print(F(","));
     } 

     Serial.println();
     Serial.print(F("PM 10 en HEX: "));
     Serial.print(buffer1[5],HEX);
     Serial.print(F(","));
     Serial.print(buffer1[6],HEX);
     Serial.println();
     Serial.print(F("PM 10 en DEC: "));
     ValuePm10=(256*buffer1[5])+buffer1[6];
     Serial.print(ValuePm10);
     Serial.println(F(" ug/m3"));
    
     Serial.println();
     Serial.print(F("PM 2.5 en HEX: "));
     Serial.print(buffer1[3],HEX);
     Serial.print(F(","));
     Serial.print(buffer1[4],HEX);
     Serial.println();
     Serial.print(F("PM 2.5 en DEC: "));
     answer=(256*buffer1[3])+buffer1[4];
     Serial.print(answer);
     Serial.println(F(" ug/m3"));
      
  break;
  case 4: //Read coeff
      Serial.println(F("---------------------"));
      Serial.println(F("Stabilize & Read Coeff..."));
      for(int j=0;j<=1;j++) // Modify "number" to stabilize measure
      {
        Serial.print(j); 
        Serial.print(F(" - "));
        SerialPM.write(readcoeff, 4);
        while(SerialPM.available()){
          Serial.print(SerialPM.read(),HEX);  // Data received and printed in the Serial Monitor.
          Serial.print(F(","));
        }
        
        Serial.println();
        delay (1000);
      }
     
      Serial.println();
      Serial.println(F("Measurement Coeff PM25"));
      SerialPM.write(readpart, 4);
      delay (500);
      do{
        buffer1[count++]=SerialPM.read();     // writing data into array   
           if(count == 8)break;
       }while(SerialPM.available() );
        
     Serial.print(F("Buffer: "));
     for(int i=0;i<5;i++)
     {
     Serial.print(buffer1[i],HEX);
     Serial.print(F(","));
     } 
     Serial.println(F(""));
    
     Serial.print(F("Coeff en HEX: "));
     Serial.println(buffer1[3],HEX);
     Serial.print(F("Coeff en DEC: "));
     answer=buffer1[3];
     Serial.println(answer);
  break;
  case 5: // Set Coeff
      Serial.println(F("---------------------"));
      Serial.println(F("Set Coeff PM25..."));
      Serial.print(F("New Coeff DEC: "));
      Serial.println(b);
      setcoeff[3]=b; //In DEC
      count =(65536-(114+b))%256;
      setcoeff[4]=count;
      Serial.print(F("4th & 5th HEX: "));
      Serial.print(setcoeff[3],HEX); 
      Serial.print(",");       
      Serial.println(setcoeff[4],HEX);   
      SerialPM.write(setcoeff, 5);
      delay(500);  
  break;
  default: //Read
      Serial.println(F("Invalid case"));
  }
 
 return answer;
  
}

int MeasureCO(byte a)
{
  
//Question & Answer Mode
const byte QuestionAnswer[] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46}; 
//Initiative Upload Mode
const byte InitiativeUpload[] = {0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};  
// Read gas concentration  
const byte ReadGas[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; 
// Sensor responses
//const byte SensorResponses[] = {0xFF, 0x86, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x20, 0x30}; 

//unsigned int answer1=0;
byte buffer1[9];
int buffer2=0;
uint8_t count = 0;
uint8_t CheckSumValue = 0;

memset(buffer1, '\0', sizeof(buffer1));


switch (a) {
case 1:  //Start   
  Serial.println(F("---------------------"));
  Serial.println(F("Setting up CO Sensor..."));
  Serial3.write(QuestionAnswer, 9);
  // Clean the buffer
   do{
     buffer1[count++]=Serial3.read();     
     if(count == 8)break;
   }while(Serial3.available() );
   
  Serial3.write(ReadGas, 9);

  do{
    buffer1[count++]=Serial3.read();     // writing data into array   
       if(count == 8)break;
   }while(Serial3.available() );

 Serial.print(F("Cleaning Buffer: "));
 for(int i=0;i<sizeof(buffer1);i++)
 {
   Serial.print(buffer1[i],HEX);
   Serial.print(F(","));
 } 
  Serial.println();
  Serial.println(F("---------------------"));
break;

case 2: //Read
  Serial.print(F("---------------------"));
  Serial.println(F("Reading from CO Sensor..."));

  do{
    buffer1[count++]=Serial3.read();     // writing data into array   
       if(count == 8)break;
   }while(Serial3.available() );

  Serial3.write(ReadGas, 9);

  Serial.print(F("Measurement CO - "));
  do{
    buffer1[count++]=Serial3.read();     // writing data into array   
       if(count == 8)break;
   }while(Serial3.available() );
   
 Serial.print(F("Buffer: "));
 for(int i=0;i<sizeof(buffer1);i++)
 {
   Serial.print(buffer1[i],HEX);
   Serial.print(F(","));
   if(i >0 && i <8)CheckSumValue+=(uint8_t)buffer1[i];
 } 
 CheckSumValue=~((uint8_t)CheckSumValue)+1;
 
// Serial.println();
// Serial.print(F("Checksum en HEX: "));
// Serial.print(CheckSumValue,HEX);

 Serial.println();
 Serial.print(F("CO en HEX: "));
 Serial.print(buffer1[6],HEX);
 Serial.print(F(","));
 Serial.print(buffer1[7],HEX);
 Serial.println();
 Serial.print(F("CO en DEC: "));
 buffer2=(256*buffer1[6])+buffer1[7];
 Serial.print(buffer2);
 Serial.println(F(" ppb"));

 if (CheckSumValue==buffer1[8])
 {
    Serial.println(F("Checksum Valido!"));
 }else{
    Serial.println(F("Checksum NO Valido!"));
    buffer2=0;
 }
 

 break;
 default:
 Serial.println(F("Invalid case"));

  }

 //Serial3.flush(); /////////////yyyyyyyyyyyyyy
 return buffer2;
}

int MeasureO3(byte a)
{
  
//Question & Answer Mode
const byte QuestionAnswer[] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46}; //Listed command to start particle measurement
//Initiative Upload Mode
const byte InititativeUpload[] = {0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};  //Listed command to read particle data
// Read gas concentration  
const byte ReadGas[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; //Listed command to stop particle measurement
// Sensor responses
//const byte SensorResponses[] = {0xFF, 0x86, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x20, 0x30}; //Listed command to stop particle

//unsigned int answer1=0;
byte buffer1[9];
int buffer2=0;
uint8_t count = 0;
uint8_t CheckSumValue = 0;

memset(buffer1, '\0', sizeof(buffer1));


switch (a) {
case 1:  //Start   
  Serial.println(F("---------------------"));
  Serial.println(F("Setting up O3 Sensor..."));
  delay(200);
  Serial1.write(QuestionAnswer, 9);
  // Clean the buffer
   do{
     buffer1[count++]=Serial1.read();     
     if(count == 8)break;
   }while(Serial1.available() );

  Serial1.write(ReadGas, 9);

  do{
    buffer1[count++]=Serial1.read();     // writing data into array   
       if(count == 8)break;
   }while(Serial1.available() );

 Serial.print(F("Cleaning Buffer: "));
 for(int i=0;i<sizeof(buffer1);i++)
 {
   Serial.print(buffer1[i],HEX);
   Serial.print(F(","));
 } 
  Serial.println();
  Serial.println(F("---------------------"));
break;

case 2: //Read
Serial.println(F("---------------------"));
Serial.println(F("Reading from O3 Sensor..."));
delay(200);

  do{
    buffer1[count++]=Serial1.read();     // writing data into array   
       if(count == 8)break;
   }while(Serial1.available() );
   
Serial1.write(ReadGas, 9);

  Serial.print(F("Measurement O3 - "));
  do{
    buffer1[count++]=Serial1.read();     // writing data into array   
       if(count == 8)break;
   }while(Serial1.available() );
   
 Serial.print(F("Buffer: "));
 for(int i=0;i<sizeof(buffer1);i++)
 {
   Serial.print(buffer1[i],HEX);
   Serial.print(F(","));
   if(i >0 && i <8)CheckSumValue+=(uint8_t)buffer1[i];
 } 
 CheckSumValue=~((uint8_t)CheckSumValue)+1;
 
// Serial.println();
// Serial.print(F("Checksum en HEX: "));
// Serial.println(CheckSumValue,HEX);

 Serial.println();
 Serial.print(F("O3 en HEX: "));
 Serial.print(buffer1[6],HEX);//4
 Serial.print(F(","));
 Serial.print(buffer1[7],HEX);//5
 Serial.println();

 Serial.print(F("O3 en DEC: "));
 buffer2=(256*buffer1[6])+buffer1[7];
 Serial.print(buffer2);
 Serial.println(F(" ppb"));

 if (CheckSumValue==buffer1[8])
 {
    Serial.println(F("Checksum Valido!"));
 }else{
    Serial.println(F("Checksum NO Valido!"));
    buffer2=0;
 }


 break;
 default:
 Serial.println(F("Invalid case"));

  }
 //Serial1.flush(); 
 return buffer2;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// uint32_t timer = millis();

void GPSFunction()
{
delay(5);
Serial.println(F("-"));
// Serial.println("gpsf");
if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  delay(5);
}

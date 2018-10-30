
#include "DHT.h"
#include <SoftwareSerial.h>
SoftwareSerial Serial2(12,11); //SIM808
SoftwareSerial Serial3(5,6); //PM2.5

#define DHTPIN A0     
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

#define sensorPin A1  // select the input pin for the CO sensor  
#define FeedbackPin A2  // select the input pin for the CO sensor feedback
#define BatteryVoltage A3  // reading battery voltage here  
#define GPSLed 7 
#define BattLed 8
#define DCSupply 10
#define GPSInput 4
#define COHeater 9
#define SIMPower 3

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long currentMillis = 60000; 
char GPS[]="19.401625,-99.147213";
    
boolean GPSFlag=0;
boolean InternetEnabled=0;
uint8_t evilcounter=0;
uint8_t countloop=0;      
 
float BattVolt=4.0;

void setup(){
  
    delay(2000);

    pinMode(BattLed, OUTPUT);     //Status Batt  
    pinMode(GPSLed, OUTPUT);     //Status GPS
    pinMode(DCSupply, INPUT);     //HIGH = connected 
    
    pinMode(SIMPower, OUTPUT);     //Power ON,OFF, command to turn the sim808 //PWR KEY
    digitalWrite(SIMPower, LOW);  //Always high  //PWR KEY
    
    analogRead(sensorPin);
    analogRead(FeedbackPin);
    
    pinMode(COHeater, OUTPUT);
    analogWrite(COHeater, 0);  // *********** turn OFF heater to approx 1,4V
    
    Serial.begin(9600);
    Serial2.begin(9600);
    Serial3.begin(9600);
        
    dht.begin();
    
   Serial.println(F("---------------------------------------"));
    Serial.println(F("Leds Starting!!!"));
    Serial.println(F("---------------------------------------"));
    
    BlinkLeds();
    
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
    
    if (digitalRead(DCSupply)) 
    {Serial.println(F("DCSupply Connected"));}
    else{
      Serial.println(F("DCSupply NOT Connected"));
      //if (BattVolt<=3.54){digitalWrite(BattLed, HIGH);BattOK=0;BattFlag=1;}else{BattOK=1;}
    }
    
    Serial.print(F("GPS ON: "));
    Serial.println(digitalRead(GPSInput));
    Serial.println(" ");

    Serial.println(F("---------------------------------------"));
    Serial.println(F("Restarting SIM!!!"));
    Serial.println(F("---------------------------------------"));

    Serial2.listen();
    BlinkLeds(); 
    close_3G(); 
    power_off(); 
    BlinkLeds(); 
    power_on();
    InternetEnabled=configure_3G(); 
    
///////////////////////////////////
    
    Serial.println(F("---------------------------------------"));
    Serial.println(F("PM2.5 Start!!!"));
    Serial.println(F("---------------------------------------"));
    Serial3.listen();
    mide25(1,0);  //Stop PM2.5
    delay(50);
    mide25(2,10);
    delay(50);
    
    if (digitalRead(GPSInput)) 
    {
      Serial.println(F("---------------------------------------"));
      Serial.println(F("GPS Start"));
      Serial.println(F("---------------------------------------"));
      BlinkLeds();
      StartGPS();
      BlinkLeds();
      Serial.println(F("---------------------------------------"));
      Serial.println(F("GPS Search!!!"));
      Serial.println(F("---------------------------------------"));  
      Serial2.listen();
      GPSFlag=SearchGPS();
    } 
  
    Serial.println(F("---------------------------------------"));   
    Serial.println(F("First measurement & loop!!!"));
    Serial.println(F("---------------------------------------"));
  

}


void loop(){

    int sensorValue=0;
    char charVal[5];

/////////////// Battery analysis

BattVolt=(2*5.0/1023)*analogRead(BatteryVoltage);
if (digitalRead(DCSupply))
{
   if (BattVolt<4.30){digitalWrite(BattLed, HIGH);} //4.28~85% 4.33~ es el max? 
   else{digitalWrite(BattLed, LOW);}    
}else{
  if (BattVolt<3.53){digitalWrite(BattLed, HIGH);} // 3.52~21% Debajo de 21 cae muy rapido = 34 mediciones mas (2 min) en 17 mediciones ya tiene warning
  else {digitalWrite(BattLed, LOW);} 
}

/////////////// Playing with GPS during execution

if(InternetEnabled)
{
  if(digitalRead(GPSInput) && !GPSFlag)
  {
    BlinkLeds();
    BlinkLeds();
    sendATcommand("AT+CGNSPWR=1", "OK", 2000);
    sendATcommand("AT+CGPSRST=1", "OK", 2000); //cold restart cold 0 hot 1 warm 2
    sendATcommand("AT+CGNSSEQ=\"RMC\"", "OK", 2000);
    GPSFlag=SearchGPS();
  }  
  if(!digitalRead(GPSInput)) 
  {
    digitalWrite(GPSLed,1);
    if(GPSFlag){
      BlinkLeds();
      BlinkLeds();
      sendATcommand("AT+CGNSPWR=0", "OK", 500);
      GPSFlag=0;
    }
  }
  
}else{
   digitalWrite(GPSLed,0);
   close_3G(); 
   delay(100);
   InternetEnabled=configure_3G(); 
}

/////////////// Indoors routine

currentMillis = millis(); 
if (currentMillis - previousMillis2 >= 60000*1 && !digitalRead(GPSInput) && InternetEnabled) {
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Indoors routine: every minute!"));
    Serial.println(F("---------------------------------------"));
    previousMillis2 = currentMillis;

   ////////////////////PM 2.5/////////////////
   
   Serial.println(F("---------------------------------------"));
   Serial.println(F("Particles PM 2.5!!!"));
   Serial.println(F("---------------------------------------"));
   
   sensorValue=0;
   memset(charVal, '\0', sizeof(charVal));
   Serial3.listen();
   sensorValue=mide25(3,28); ////////// 28

   Serial.print(F("---> Medida de Particulas PM 2.5: "));
   Serial.print(sensorValue);
   Serial.print(F(" ug/m3"));
   Serial.println();
   
   dtostrf(sensorValue, 3, 0, charVal);  //mininum width, precision; first string copied into last array
   
   Serial2.listen();
   GETSend(charVal,"pm25","0");

  ////////////////////Humidity & Temp/////////////
        
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Humidity & Temperature!!!"));
    Serial.println(F("---------------------------------------"));
    sensorValue=0;
    memset(charVal, '\0', sizeof(charVal));
    

    float fh = dht.readHumidity();
    float ft = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(ft) || isnan(fh)) 
    {
        Serial.println(F("Failed to read from DHT"));
    } 
    else 
    {
        Serial.print(F("---> Humidity: ")); 
        Serial.print(fh);
        Serial.println(F(" %\t"));
        Serial.print(F("---> Temperature: ")); 
        Serial.print(ft);
        Serial.println(F(" *C"));
    }

     dtostrf(fh, 5, 2, charVal);  //mininum width, precision; first string copied into last array
     
     GETSend(charVal,"humidity","0");

     dtostrf(ft, 5, 2, charVal);  //mininum width, precision; first string copied into last array
    
     GETSend(charVal,"temperature","0");
    //if (BattVolt<=3.73 && BattVolt>3.54){digitalWrite(BattLed, HIGH);delay(1000);digitalWrite(BattLed, LOW);} //3.73V ~40%
    
    ////////////////////Battery value ////////////// 

    Serial.println(F("---------------------------------------"));
    Serial.println(F("Battery Level!!!"));
    Serial.println(F("---------------------------------------"));

   sensorValue=0;
   memset(charVal, '\0', sizeof(charVal));
    
   sensorValue=BatteryLevel();

   Serial.print(F("---> Bateria: "));
   Serial.print(sensorValue);
   Serial.println();
   
   dtostrf(sensorValue, 3, 0, charVal);  //mininum width, precision; first string copied into last array
   
   GETSend(charVal,"battery","0");
   //if (BattVolt<=3.73 && BattVolt>3.54){digitalWrite(BattLed, HIGH);delay(1000);digitalWrite(BattLed, LOW);} //3.73V ~40%
  
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Checking Parameters"));
  Serial.println(F("---------------------------------------"));
   
  Serial.print(F("Battery Voltage: ")); 
  Serial.print(BattVolt);
  Serial.println(F(" V"));

  Serial.print(F("GPS: "));
  Serial.println(digitalRead(GPSInput));
 
  Serial.print(F("DC Connection: "));
  Serial.println(digitalRead(DCSupply)); 
   
  Serial.println(F("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooo"));
   
 };

////////////////////Outdoor routine ////////////// 

    if ((currentMillis - previousMillis >= 4000) && GPSFlag && InternetEnabled) {  //yyy GPSFlag OR digitalRead(GPSInput)
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Measurement every 5 secs..."));
    Serial.println(F("---------------------------------------"));
    
    previousMillis = currentMillis;
  

////////////////////PM 2.5/////////////////

   Serial.println(F("---------------------------------------"));
   Serial.println(F("Particles PM 2.5!!!"));
   Serial.println(F("---------------------------------------"));
   
   sensorValue=0;
   memset(charVal, '\0', sizeof(charVal));
   Serial3.listen();
   sensorValue=mide25(3,28); ////////// 0.5 seg

   Serial.print(F("---> Medida de Particulas PM 2.5: "));
   Serial.print(sensorValue);
   Serial.print(F(" ug/m3"));
   Serial.println();
   
   dtostrf(sensorValue, 3, 0, charVal);  //mininum width, precision; first string copied into last array
   
   Serial2.listen();
   //GETSend(charVal,"pm25","");***************
   
 ////////////////////GPS String //////////////
  
    Serial.println(F("---------------------------------------"));
    Serial.println(F("GPS"));
    Serial.println(F("---------------------------------------"));
  //Serial.println(F("Determining GPS"));
  
    memset(GPS,'\0', sizeof(GPS));
    memcpy(GPS,GPS_String(),20);
  
    Serial.print(F("---> GPS: "));
    Serial.println(GPS);
    
    if (strstr(GPS,",,") != NULL) /// yyyyyyyy I need to catch the "      " here 
    {
    //it finds the 2nd string in 1st string, it matches 
    Serial.println(F("This GPS is invalid")); 
    }else{
      if (strstr(GPS,"000") != NULL)
      {Serial.println(F("This GPS is invalid")); }
      else
      {GETSend(GPS,"location",charVal);}
    }   
    ////////////////////End of Concatenating ALL/////////////
  
     Serial.println(F("**************************************************"));
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
  Serial.println(F("Turning SIM900 ON...")); 
  delay(50);
  if (sendATcommand("AT+CGMM=?", "OK", 100) == 1)
    {// Nothing, the SIM900 is ON
    Serial.println(F("SIM Already ON!"));
    }
    else
    {  
      digitalWrite(SIMPower,HIGH);
      delay(1400);
      digitalWrite(SIMPower,LOW);
    }
  
  BlinkLeds(); 
  sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications
  
  countloop=0;    
  while( (sendATcommand("AT+CREG?", "+CREG: 1,1", 600) // before 0,1
            || sendATcommand("AT+CREG?", "+CREG: 0,5", 600)
            || sendATcommand("AT+CREG?", "+CREG: 0,1", 600)
            || sendATcommand("AT+CREG?", "+CREG: 2,1", 600)) == 0 )
            {countloop++; BlinkLeds(); if(countloop >= 8){break;}};  //YYYYY If it does not find network think what to do
            
  sendATcommand("AT+CREG=0", "OK", 5000); // Deactivating CREG notifications
  BlinkLeds();
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

int BatteryLevel(){
  char StringResult[40];   // BEFORE IT WAS 15 o 60
  char StringResult2[3];
  char* pch;
  memset(StringResult, '\0', 40);
  memset(StringResult2, '\0', 3);
 
  uint8_t StartIndex=11;//16
  uint8_t EndIndex=25; //26
  
  sendATcommand("AT+CBC", "OK", 5000);
  
  memcpy(StringResult,RetrieveATResponse("AT+CBC",StartIndex,EndIndex,15000,"OK"),EndIndex-StartIndex);
  
  Serial.println(F("First String:"));
  Serial.println(StringResult);
 
  pch= strstr(StringResult, "CBC:");
  memcpy(StringResult2,pch+7,3);
  
  Serial.println(F("Second String:"));
  Serial.println(StringResult2);
  
  int x=atoi(StringResult2);
  
  Serial.println(F("Third String: "));
  Serial.println(x);
    
  return x; 
  
}

boolean configure_3G(){
    boolean InternetOn=false;
    char aux_str[45]; 
    
    Serial.println(F("Configuring 3G Connection...")); 
 
    sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
    
    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
    sendATcommand(aux_str, "OK", 2000);
    
    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
    sendATcommand(aux_str, "OK", 2000);
    BlinkLeds();
    snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
    sendATcommand(aux_str, "OK", 2000);
    BlinkLeds(); 

    countloop=0;
    while ( (  sendATcommand("AT+SAPBR=1,1", "OK", 6000)
            || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 6000) ) != 1)
           {countloop++;BlinkLeds();if(countloop >= 10){break;}}; 
  
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


void GETSend(char* value, char* variable, char* value2){
  
    char aux_str[270];  
    memset(aux_str, '\0', 270);
    //char php_script[]="push2gas.com/dashboard/csv2sql.php";
    //sendATcommand("", "", 1000); // useless

     if(variable=="location"){
     Serial.println(F("GETSend for Location enabled"));
     //                                                        http://www.smability.com/airquality/airdata.php?deviceID=0001&reading=%2219.411531,-99.170450%22&alarm=4&type=location
     //snprintf(aux_str, sizeof(aux_str), "AT+HTTPPARA=\"URL\",\"http://www.smability.com/airquality/airdata.php?deviceID=0002%sreading=\"%s\"%salarm=05%stype=location\"","&",value,"&","&");
     snprintf(aux_str, sizeof(aux_str), "AT+HTTPPARA=\"URL\",\"http://www.smability.com/airquality/airdata.php?deviceID=03%sreading=%s%s%s%salarm=%s%stype=location\"","&","%22",value,"%22","&",value2,"&");
     sendATcommand(aux_str, "OK", 520); //yyyy 350
     
     countloop=1;                                           
     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,34", 11000) != 1) //45 is the lenght of char text from file csv2sql
           {delay(3);if(countloop >= 120){break;};countloop++;}; //50
     
     if(countloop >=120 ){
         Serial.println(F("Location Wrong Happened - No Reset - Just wait"));
         evilcounter=evilcounter+1;
       }
       
       if(evilcounter >= 6){
         Serial.println(F("Location Wrong Happened - SEVERAL TIMES - 2G Reset"));
         digitalWrite(GPSLed,0);
         InternetEnabled=0;
         
         while(!InternetEnabled){
           if(evilcounter >= 10){
             power_off(); 
             delay(50);
             power_on(); 
           }
           close_3G(); 
           delay(50);
           InternetEnabled=configure_3G();
           evilcounter=evilcounter+1;
           if(evilcounter >= 50){break;};
         }
         
         GPSFlag=0;
         evilcounter=0;
       }
   
     }else{
     //snprintf(aux_str, sizeof(aux_str), "AT+HTTPPARA=\"URL\",\"http://www.smability.com/airquality/airdata.php?deviceID=0002%sreading=%s%salarm=%s%stype=%s\"","&",value,"&",value2,"&",variable);
     snprintf(aux_str, sizeof(aux_str), "AT+HTTPPARA=\"URL\",\"http://www.smability.com/airquality/airdata.php?deviceID=03%sreading=%s%salarm=00%stype=%s\"","&",value,"&","&",variable);
     sendATcommand(aux_str, "OK", 520); 
     
     countloop=1;                                           
     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,34", 11000) != 1) //45 is the lenght of char text from file csv2sql
           {delay(3);if(countloop >= 120){break;};countloop++;}; 
     
     if(countloop >= 120){
         Serial.println(F("Something Wrong Happened - Big reset"));
         digitalWrite(GPSLed,0);
         InternetEnabled=0;
         while(!InternetEnabled){
           close_3G();
           power_off(); 
           delay(50);
           power_on(); 
           InternetEnabled=configure_3G();
         }
         GPSFlag=0;
       }
     
     }
 
     if(sendATcommand("AT+HTTPREAD", "!!", 1200)) // Move inside
     {
       Serial.println(F("Success in uploading data"));
       evilcounter=0;
       for(int j=0;j<6;j++)
       {
          digitalWrite(GPSLed, !digitalRead(GPSLed));
          delay(80);
       }       
     }
                              
     //sendATcommand("AT+HTTPREAD", "!!", 8000);      
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[270];
    unsigned long previous;

    memset(response, '\0', 270);    // Initialize the string

    delay(10);

    while( Serial2.available() > 0) Serial2.read();    // Clean the input buffer

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
/*
int8_t sendATcommandDebug(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[180];
    unsigned long previous;

    memset(response, '\0', 180);    // Initialize the string

    delay(50);

    while( Serial2.available() > 0) Serial2.read();    // Clean the input buffer

    Serial2.println(ATcommand);    // Send the AT command

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(Serial2.available() != 0){ 
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = Serial2.read();
            
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
  
  Serial2.println(ATcommand);       // write it to the GPRS shield
  Serial.println();
  do{
    if (Serial2.available())              // if date is comming from softwareserial port ==> data is comming from gprs shield
    {

       buffer1[count++]=Serial2.read();     // writing data into array
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
 memcpy(StringResult, buffer2, FinalIndex-InitialIndex);
 
 count = 0;                       // set counter of while loop to zero
    
 return StringResult;
  
 }

void BlinkLeds()
{ 
    boolean templed1=digitalRead(BattLed);
    boolean templed2=digitalRead(GPSLed);
    
    digitalWrite(BattLed, LOW);
    digitalWrite(GPSLed, LOW);
    delay(250);
    digitalWrite(BattLed, HIGH);
    digitalWrite(GPSLed, HIGH);
    delay(250);
    digitalWrite(BattLed, LOW);
    digitalWrite(GPSLed, LOW);
    delay(250);
    digitalWrite(BattLed, HIGH);
    digitalWrite(GPSLed, HIGH);
    delay(250);
    
    digitalWrite(BattLed, templed1);
    digitalWrite(GPSLed, templed2); 
}

void StartGPS(){ //3:16pm
  sendATcommand("AT+CGNSPWR=0", "OK", 2000);
  sendATcommand("AT+CGNSPWR=1", "OK", 2000);
  sendATcommand("AT+CGPSRST=0", "OK", 2000); //cold restart cold 0 hot 1 warm 2
  sendATcommand("AT+CGNSSEQ=\"RMC\"", "OK", 2000);
  sendATcommand("AT+CGPSSTATUS?", "Fix", 2000);
}

boolean SearchGPS(){
  int countloop=1;
  boolean GPSFound=0;  
  while (sendATcommand("AT+CGPSSTATUS?", "3D Fix", 1000) != 1)
  {
    Serial.print(F("Searching Satellites (Stop at 600 seg): "));
    Serial.println(countloop);
    if(countloop == 300){sendATcommand("AT+CGPSRST=2", "OK", 2000);} //at half warm restart
    if(countloop >= 600 || !digitalRead(GPSInput)){break;} //600 veces 1 seg = 10 min 
    countloop++;
    delay(250); 
    digitalWrite(GPSLed, HIGH);
    delay(250);
    digitalWrite(GPSLed, LOW);
  };
  Serial.println();
  if(countloop < 600){Serial.println(F("GPS Satellites Found"));digitalWrite(GPSLed, HIGH);GPSFound=1;}
  if(countloop >= 600){Serial.println(F("GPS Satellites NOT Found"));digitalWrite(GPSLed, LOW);GPSFound=0;}
  if(!digitalRead(GPSInput)){Serial.println(F("GPS Search Interrupted"));digitalWrite(GPSLed, LOW);GPSFound=0;}
  return GPSFound;
}

char* GPS_String(){  

  char StringResult[100];   
  memset(StringResult, '\0', 100);
  int countloop=1;
  
  while (sendATcommand("AT+CGPSSTATUS?", "D Fix", 1000) != 1) // Faster if lost, last reference
    {
    Serial.print(F("Searching Satellites (Stop at 30): "));
    Serial.println(countloop);  
    //if(countloop == 20){sendATcommand("AT+CGPSRST=1", "OK", 2000);} //at half hot restart
    if(countloop >= 30){break;} //60 veces 1 seg = 1 min //************60
    countloop++;
    delay(250); // ****** it has to be 430
    digitalWrite(GPSLed, HIGH);
    delay(250);
    digitalWrite(GPSLed, LOW);
  }; 
  if(countloop < 30){Serial.println(F("GPS Satellites Found"));digitalWrite(GPSLed, HIGH);}
  if(countloop >= 30){Serial.println(F("GPS Satellites NOT Found"));digitalWrite(GPSLed, LOW);}

  sendATcommand("AT+CGNSINF", "OK", 500);  

  uint8_t StartIndex=46; //44,48
  uint8_t EndIndex=66;  //65,68
      
  memcpy(StringResult,RetrieveATResponse("AT+CGNSINF",StartIndex,EndIndex,30000,"OK"),EndIndex-StartIndex); 
     
  Serial.println(F("GPS String"));
  Serial.println(StringResult);
    
  return StringResult;
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
  
  //byte setcoeff[] = {0x68, 0x02, 0x08, 0x64, 0x2A}; //Stop auto send
  //const byte readcoeff[] = {0x68, 0x01, 0x10, 0x87}; //Stop auto send
  
  unsigned int answer1=0;

  byte buffer1[10];
  int buffer2[3];
  uint8_t count = 0;
  memset(buffer1, '\0', sizeof(buffer1));
  memset(buffer2, '\0', sizeof(buffer2));
  
  switch (a) {
  case 1:  //Stop    
    Serial.println(F("---------------------"));
    Serial.println(F("Disable & Stop PM25..."));
    Serial3.write(readstop, 4);
    delay(500);  
  break;
  case 2:  //Start
    Serial.println(F("---------------------"));
    Serial.println(F("Start & Disable & Pre-stabilize PM25..."));
    Serial3.write(readstart, 4);
    delay(500);
    Serial3.write(disableauto, 4);
    for(int j=0;j<=b;j++) // Modify "number" to stabilize measure
      {
        Serial.print(j); 
        Serial.print(F(" - "));
        Serial3.write(readpart, 4);
        while(Serial3.available()){
          Serial.print(Serial3.read(),HEX);  // Data received and printed in the Serial Monitor.
          Serial.print(F(","));
        }
        Serial.println();
        BlinkLeds();
      }

    break;
  case 3:  //Read
      Serial.println(F("---------------------"));
      Serial.println(F("Stabilize & read PM25..."));
      
      for (int m=1;m<=3;m++)
      {
        memset(buffer1, '\0', sizeof(buffer1));
        count = 0;
        Serial.println();
        Serial.print(F("---------- "));
        Serial.print(F("Measurement PM25 - "));
        Serial.println(m);

        Serial3.write(readpart, 4);
        delay(20);
        do{
          buffer1[count++]=Serial3.read();     // writing data into array   
             if(count == 8)break;
         }while(Serial3.available() );
         
       Serial.print(F("Buffer: "));
       for(int i=0;i<sizeof(buffer1);i++)
       {
       Serial.print(buffer1[i],HEX);
       Serial.print(F(","));
       } 
      
       Serial.println();
       Serial.print(F("PM 2.5 en HEX: "));
       Serial.print(buffer1[3],HEX);
       Serial.print(F(","));
       Serial.print(buffer1[4],HEX);
       Serial.println();
       Serial.print(F("PM 2.5 en DEC: "));
       buffer2[m]=(256*buffer1[3])+buffer1[4];
       Serial.print(buffer2[m]);
       Serial.println(F(" ug/m3"));
       delay(1); // configure here for slower applications       
      }
       Serial.println(F("---------- "));
       Serial.print(F("PM 2.5 recolectados: "));
       Serial.print(buffer2[1]);
       Serial.print(F(","));
       Serial.print(buffer2[2]);
       Serial.print(F(","));
       Serial.println(buffer2[3]);
       Serial.print(F("PM 2.5 average:"));
       answer1=floor((buffer2[1]+buffer2[2]+buffer2[3])/3);
       Serial.println(answer1);
       Serial.println(F("---------- "));
       
  break;
  /*
  case 4: //Read coeff
      Serial.println(F("---------------------"));
      Serial.println(F("Stabilize & Read Coeff..."));
      for(int j=0;j<=1;j++) // Modify "number" to stabilize measure
      {
        Serial.print(j); 
        Serial.print(F(" - "));
        Serial3.write(readcoeff, 4);
        while(Serial3.available()){
          Serial.print(Serial3.read(),HEX);  // Data received and printed in the Serial Monitor.
          Serial.print(F(","));
        }
        Serial.println();
        delay (1000);
      }
     
      Serial.println();
      Serial.println(F("Measurement Coeff PM25"));
      Serial3.write(readpart, 4);
      delay (500);
      do{
        buffer1[count++]=Serial3.read();     // writing data into array   
           if(count == 8)break;
       }while(Serial3.available() );
        
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
     answer1=buffer1[3];
     Serial.println(answer1);
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
      Serial3.write(setcoeff, 5);
      delay(500);  
  break;
  */
  default: //Read
      Serial.println(F("Invalid case"));
  }
 
 return answer1;
  
}

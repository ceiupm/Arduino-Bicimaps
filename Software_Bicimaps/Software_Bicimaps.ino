

#include <Sensores.h>
#include <SoftwareSerial.h>
#define INTERVALO 3000

//SoftwareSerial DSSerial(8,9);

unsigned long time = 0;
unsigned long timeOut;
boolean reading = false;
long endTime = 0;
char reader;
unsigned long endReading;


#define FIVE_MIN 300000
#define TIMEOUT_MS 1000
#define waitUntilTimeOut(C) timeOut = millis() + TIMEOUT_MS; while((C) && (millis() < timeOut));

//Create an instance of software serial
SoftwareSerial hpmaSerial(9, 8); // Feather TX, Feather RX
//Create an instance of the hpma115S0 library
HPMA115S0 hpma115S0(hpmaSerial);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  hpmaSerial.begin(9600);
  delay(3000);
  hpma115S0.StopParticleMeasurement();
  while(Serial.available() > 0){
    Serial.read();
  }
}

void loop() { 
  
  waitUntilTimeOut(Serial.available() == 0);
 
  if(Serial.available()>0){
    reader = Serial.read();
    switch(reader){
      case 'p': {
        reading = true;      
        waitUntilTimeOut(Serial.available() == 0);
        String endTimeString = Serial.readStringUntil('\n');
        endTime = endTimeString.toInt();
        //Serial.print(endTimeString);
        endReading = millis() + endTime + FIVE_MIN;
        hpma115S0.Init();
        break;
      }   
      case 's':{
        hpma115S0.StopParticleMeasurement();
        endTime = 0;
        reading = false;
        break;
      }
      case 'd':{
        reading = true; 
        hpma115S0.Init();        
      }
      //default: 
        //Serial.read();
    }
  }
  if(reading){
    if( (reader == 'p' && millis() < endReading) || reader == 'd'){
      unsigned int pm2_5, pm10;
      if (hpma115S0.ReadParticleMeasurement(&pm2_5, &pm10)) {
        //Serial.println("PM 2.5: " + String(pm2_5) + " ug/m3" );
        //Serial.println("PM 10: " + String(pm10) + " ug/m3" );
        Serial.println(String(pm2_5)+" "+String(pm10));
      } 
      //Serial.println(myNumber);
      
      delay(2000); 
    } else {
      hpma115S0.StopParticleMeasurement();
      endTime = 0;
      reading = false;
      
    }
  }
}

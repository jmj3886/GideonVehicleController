#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <OBD.h>

COBDI2C        obd;
bool           hasMEMS;
SoftwareSerial bluetooth(7, 8); // RX, TX

// Output
const int      redPin    = 6;  // Red LED,   connected to digital pin 5
const int      greenPin  = 5;  // Green LED, connected to digital pin 6
const int      bluePin   = 4;  // Blue LED,  connected to digital pin 3

const byte     RED[]     = {255,0,0}; 
const byte     ORANGE[]  = {255, 10, 0}; 
const byte     YELLOW[]  = {255,255,0}; 
const byte     GREEN[]   = {0,255,0}; 
const byte     BLUE[]    = {0,0,255}; 
const byte     PINK[]    = {255,20,147};
const byte     OFF[]     = {0,0,0};
long int       inByte; 
unsigned long  wait      = 500;
String         selection = "YELLOW\n";
boolean        didInit = false;

void setup()
{
  selection = "BLUE\n";
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  pinMode(redPin,   OUTPUT);   // sets the pins as output
  pinMode(greenPin, OUTPUT);   
  pinMode(bluePin,  OUTPUT); 
   
  delay(wait);
  
  // initialize OBD-II adapter
  obd.begin();
  hasMEMS = obd.memsInit();
  unsigned long startMillis = millis();
  Serial.println("Init...");
  do 
  {
    //didInit = obd.init();
  } while ((!didInit) && ((millis() - startMillis) < wait));

  if (didInit)
  {
    char buf[64];
    if (obd.getVIN(buf, sizeof(buf))) {
        Serial.print("VIN:");
        Serial.println(buf);
    }
  }
  else
  {    
    Serial.println("Init - (FAIL)");
  }
  
  delay(3000);
}

void readMEMS()
{
    int acc[3];
    int gyro[3];
    int temp;

    if (!obd.memsRead(acc, gyro, 0, &temp)) return;

    //Serial.print("ACC:");
    //Serial.print(acc[0]);
    //Serial.print('/');
    //Serial.print(acc[1]);
    //Serial.print('/');
    //Serial.print(acc[2]);

    //Serial.print(" GYRO:");
    //Serial.print(gyro[0]);
    //Serial.print('/');
    //Serial.print(gyro[1]);
    //Serial.print('/');
    //Serial.print(gyro[2]);

    float fTemp = (float)temp / 10;
    fTemp = (fTemp * 1.8)+32;    
    String command = "C"+String(fTemp);
    while(command.length() < 4){
      command += ".";
    }
    bluetooth.print(command);
}

void readPIDs()
{
    static const byte pidlist[] = {PID_SPEED, PID_FUEL_LEVEL, PID_AMBIENT_TEMP};
    for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) 
    {
        byte pid = pidlist[i];
        bool valid = obd.isValidPID(pid);
        char sendMarker;
        int value;
        if (valid) 
        {
            if (obd.readPID(pid, value)) 
            {
              switch(pid)
              {
                case PID_SPEED:
                  sendMarker = 'S';
                  value = value *(1.0/1.6093440);
                  break;
                case PID_FUEL_LEVEL:
                  sendMarker = 'F';
                  break;
                case PID_AMBIENT_TEMP:
                  float fTemp = (float)value;
                  fTemp = (fTemp * 1.8)+32;
                  value = fTemp;
                  sendMarker = 'A';
                  break;
              }
              String command = sendMarker+String(value);
              while(command.length() < 4)
              {
                command += ".";
              }
              bluetooth.print(command);
            }
        }        
     }
}

void readBatteryVoltage()
{
  bluetooth.print("B"+String(obd.getVoltage()));
}

void outputColour(int red, int green, int blue) 
{
  analogWrite(redPin, red);
  analogWrite(bluePin, blue);
  analogWrite(greenPin, green);    
}

// Main program
void loop()
{
  if(bluetooth.available())
  {
    selection = bluetooth.readString();
    Serial.println(selection);
  }
  
  byte COLOR[] = {0,0,0};
  if (selection == "BLUE\n"){
    memcpy( COLOR, BLUE, 3 );
  }else if (selection == "GREEN\n"){
    memcpy( COLOR, GREEN, 3 );
  }else if (selection == "RED\n"){
    memcpy( COLOR, RED, 3 );
  }else if (selection == "PINK\n"){
    memcpy( COLOR, PINK, 3 );
  }else if (selection == "YELLOW\n"){
    memcpy( COLOR, YELLOW, 3 );
  }else if (selection == "ORANGE\n"){
    memcpy( COLOR, ORANGE, 3 );
  }else if (selection == "OFF\n"){
    memcpy( COLOR, OFF, 3 );
  }

  delay(100);
  
  outputColour(COLOR[0], COLOR[1], COLOR[2]);  

  if (didInit)
  {
    readBatteryVoltage();
    readPIDs();
    unsigned int codes[6];
    byte dtcCount = obd.readDTC(codes, 6);
    if(dtcCount > 0)
    {
      bluetooth.print("M"+String(dtcCount)+"..");
    }
  }
  if (hasMEMS) 
  {
    readMEMS();
  }
  delay(wait);
}


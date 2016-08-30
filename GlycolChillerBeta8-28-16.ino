#include <OneWire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PID_v1.h>        
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#define heaterRelayPin A3
#define ambientTempProbe 22
#define coolantTempProbe 23
#define condensorTempProbe 24
#define cSetTempPot A2
#define fv1TempProbe 25
#define fv2TempProbe 26
#define fv1SetTempPOT A0 // input pin for the potentiometer
#define fv2SetTempPOT A1 // input pin for the potentiometer


double Setpoint; //PID setpoint
double Input1; //Temperature input to the PID statement (from coolantTemp index)
double Input2; //Temperature input to the PID statement (from fv1Temp index)
double Input3; //Temperature input to the PID statement (from fv2Temp index)
double Output1;
double Output2;
double Output3;

//variables to store loop data
float aTemp;
float cTemp;
float cSetTemp;
float fv1Temp;
float fv1SetTemp;
float fv2Temp;
float fv2SetTemp;

//variables to stoare incoming value from temp sensors
int cSensorValue; // coolant
int fv1SensorValue; // fv1
int fv2SensorValue;// fv2


//Specify the links and initial tuning parameters for the PID
float Kp = 20; //was 10,15
float Ki = 1.5; //was 2, 1.2
float Kd = 0.5; //was .2

PID cPID(&Input1, &Output1, &Setpoint, Kp, Ki, Kd, DIRECT);
PID fv1PID(&Input2, &Output2, &Setpoint, Kp, Ki, Kd, DIRECT);
PID fv2PID(&Input3, &Output3, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

//Variables for smoothing the INPUT temperatures
const int numReadings = 5;
double readings[numReadings]; // the readings from the analog input
int index = 0; // the index of the current reading
double total = 0; // the running total
double input = 0; // the average


//Create OneWire protocol, relate it to input pin
OneWire ds1(ambientTempProbe);
OneWire ds2(coolantTempProbe);
OneWire ds3(fv1TempProbe);
OneWire ds4(fv2TempProbe);

//Use DallasTemperature.h to get the sensor readings
DallasTemperature ambientTempSensor(&ds1);
DallasTemperature coolantTempSensor(&ds2);
DallasTemperature fv1TempSensor(&ds3);
DallasTemperature fv2TempSensor(&ds4);

// Enter a MAC address for your ethernet shield
byte mac[] = { 0x90, 0xa4, 0xda, 0x00, 0xa5, 0x59 };


IPAddress server(64, 90, 34, 142); // IP of fermchamber.mavenbrewing.com (new 5/2016) 64.90.34.142

// Initialize the Ethernet client library
EthernetClient client;

//Run once at startup
void setup(void) {

pinMode(A3, OUTPUT);
pinMode(ambientTempProbe, INPUT);
pinMode(coolantTempProbe, INPUT);
pinMode(fv1TempProbe, INPUT);
pinMode(fv2TempProbe, INPUT);
pinMode(cSetTempPot, INPUT);
pinMode(fv1SetTempPOT, INPUT);
pinMode(fv2SetTempPOT, INPUT);
digitalWrite(A3, HIGH); //initializes heater relay OFF
ambientTempSensor.begin();
coolantTempSensor.begin();
fv1TempSensor.begin();
fv2TempSensor.begin();

windowStartTime = millis();

Serial.begin(9600);

//tell the PID to range between 0 and the full window size
cPID.SetOutputLimits(0, WindowSize);
fv1PID.SetOutputLimits(0, WindowSize);
fv2PID.SetOutputLimits(0, WindowSize);

//turn the PID on
cPID.SetMode(AUTOMATIC);
fv1PID.SetMode(AUTOMATIC);
fv2PID.SetMode(AUTOMATIC);


analogReference(EXTERNAL);

}

void loop(void) {
  
//get temps and setpoints
getAmbientTemp();
getCoolantTemp();
getCoolantSetpoint();
getFV1Temp();
getFV1Setpoint();
getFV2Temp();
getFV2Setpoint();


analogReference(DEFAULT);

cPID.Compute();  // send it to the computer as ASCII digits
fv1PID.Compute();  // send it to the computer as ASCII digits
fv2PID.Compute();  // send it to the computer as ASCII digits


// turn the output pin on/off based on pid output
if(millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
windowStartTime += WindowSize;
  }
if(Output1*1000 > millis() - windowStartTime)
    {
  digitalWrite(heaterRelayPin, LOW);
 // setBacklight(255 , 0 , 0); //RED
  Serial.println("HEATING");
  
  }

else 
    {
digitalWrite(heaterRelayPin, HIGH);
//setBacklight(0 , 255 , 0); //GREEN
Serial.println("RESTING");
  }


}


float getAmbientTemp(){
  ambientTempSensor.requestTemperatures();
  float aTempSense = ambientTempSensor.getTempFByIndex(0);
  float aTemp = aTempSense;
  Serial.println(aTemp);
  return aTemp;
}

float getCoolantTemp(){
  coolantTempSensor.requestTemperatures();
  float coolantTempSense = coolantTempSensor.getTempFByIndex(0);
  float cTemp = coolantTempSense;
  Serial.println(cTemp);
  return cTemp;
}

float getFV1Temp(){
  fv1TempSensor.requestTemperatures();
  float fv1TempSense = fv1TempSensor.getTempFByIndex(0);
  float fv1Temp = fv1TempSense;
  Serial.println(fv1Temp);
  return fv1Temp;
}

float getFV2Temp(){
  fv2TempSensor.requestTemperatures();
  float fv2TempSense = fv2TempSensor.getTempFByIndex(0);
  float fv2Temp = fv2TempSense;
  Serial.println(fv2Temp);
  return fv2Temp;
}

float getCoolantSetpoint(){
 cSensorValue = analogRead(cSetTempPot);
float Setpoint1 = map(cSensorValue, 0, 1023, 50, 80);
float cSetTemp = Setpoint1;
analogReference(EXTERNAL);
total = total - readings[index]; 
float cTemp = getCoolantTemp();
readings[index] = cTemp;
total = total + readings[index]; 
index = index + 1; 
if (index >= numReadings) // if we're at the end of the array...
index = 0;  // ...wrap around to the beginning:
Input1 = total / numReadings; // calculate the average:
return Input1;
}

float getFV1Setpoint(){
  fv1SensorValue = analogRead(fv1SetTempPOT);
float Setpoint2 = map(fv1SensorValue, 0, 1023, 50, 80);
float fv1SetTemp = Setpoint2;
analogReference(EXTERNAL);
total = total - readings[index]; 
float fv1Temp = getFV1Temp();
readings[index] = fv1Temp;
total = total + readings[index]; 
index = index + 1; 
if (index >= numReadings) // if we're at the end of the array...
index = 0;  // ...wrap around to the beginning:
Input2 = total / numReadings; // calculate the average:
return Input2;
}

float getFV2Setpoint(){
  fv2SensorValue = analogRead(fv2SetTempPOT);
float Setpoint3 = map(fv2SensorValue, 0, 1023, 50, 80);
float fv2SetTemp = Setpoint3;
analogReference(EXTERNAL);
total = total - readings[index]; 
float fv2Temp = getFV2Temp();
readings[index] = fv2Temp;
total = total + readings[index]; 
index = index + 1; 
if (index >= numReadings) // if we're at the end of the array...
index = 0;  // ...wrap around to the beginning:
Input3 = total / numReadings; // calculate the average:
return Input3;
}

/*
  void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
 // g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);
 
  analogWrite(REDLIGHT, r);
  analogWrite(GREENLIGHT, g);
  analogWrite(BLUELIGHT, b);

}
*/

/*
void sendViaEthernet()
{
  Ethernet.begin(mac); 
  //delay(100); 
  
  float fcTemp = getTemp()*9/5+32;
  float aTemp = getAmbientTemp();
  
  Serial.println("Connecting...");  
 
 if (client.connect(server, 80))
  {
    Serial.println("Connected."); // if connected, report back via serial
    // Make an HTTP request:
    client.print("GET http://fermchamber.mavenbrewing.com/data.php?aTemp="); //PHP file that posts to the database
    client.print(aTemp);
    client.print("&fcTemp=");
    client.print(fcTemp);
    client.print("&setTemp=");
    client.print(setTemp);
    client.println(" HTTP/1.1"); //ends the session with the PHP file
    client.println("Host: http://www.fermchamber.mavenbrewing.com"); //closes out session
    client.println();
     client.stop();
  }  
}



 //end of Ethernet PHP
 */


/****************** Maven Fermentation Glycol Chiller**************************
** Two plastic fermentors with copper coils wrapped around and insulated     **
** Thermon heating wire wrapped around the copper coils to distribute heat   **
** 12VDC electric ball valves require an open signal and close signal        **
** 120VAC pump triggered from SSR direct from digital pin when a valve opens **
******************************************************************************/


#include <OneWire.h>
#include <Ethernet.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <SPI.h>

#define loopSpeedLED 13 //Loop speed monitor LED
#define fv1SetTempPot A0 // input pin for the potentiometers
#define fv2SetTempPot A1 //
#define cSetTempPot A2 //
#define fv1HeatPin 42 //fv2 heater relay
#define fv2HeatPin 43 //fv1 heater relay
#define ambientTempProbe 22 // input pin for DS18B20 temperature probes
#define coolantTempProbe 23 //
#define condensorTempProbe 24 //
#define fv1TempProbe 25 //
#define fv2TempProbe 26 //
#define fv1ValveO 30 //output to darlington transistor to open valve (O)
#define fv1ValveC 31 //output to darlington transistor to close valve (C)
#define fv2ValveO 35 //
#define fv2ValveC 36 //
#define chillerPin 41 // output pin to chiller SSR
#define coolantPump 40 // output pin to coolant pump
#define fv1CrashSwitch 32
#define fv1CrashSwitch 37


double Setpoint1; //FV1 PID setpoint)
double Setpoint2; //FV2 PID setpoint)
double Input1; //Temperature input to the PID statement (from fv1Temp index)
double Input2; //Temperature input to the PID statement (from fv2Temp index)
double Output1;
double Output2;

//variables to store loop data
float aTemp;
float cTemp;
int cSetTemp = 30;
float fv1Temp;
float fv1SetTemp;
float fv2Temp;
float fv2SetTemp;
unsigned long chillerOnTime;
unsigned long chillerOffTime;
unsigned long time;
int valveOnInterval = 10000; //valve on for 10 seconds
int chillerOffInterval = 600000; //wait 10 minutes before turning on chiller
bool FV1COOL;
bool FV1HEAT;
bool FV1IDLE;
bool FV2COOL;
bool FV2HEAT;
bool FV2IDLE;
unsigned char fv1State;
unsigned char fv2State;
unsigned long updateTimer = 0;
unsigned long stateTimer = 0;


//variables to stoare incoming values from temp sensors
int cSensorValue; // coolant
int fv1SensorValue; // fv1
int fv2SensorValue;// fv2
float chillerHysterisis = 0.5;
float fvHysterisis = 0.5;


//Specify the links and initial tuning parameters for the PID
float Kp = 20;
float Ki = 1.5;
float Kd = 0.5;

PID fv1PID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID fv2PID(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

//Variables for smoothing the INPUT temperatures
const int numReadings = 5;
float readings[numReadings]; // the readings from the analog input
int index = 0; // the index of the current reading
float total = 0; // the running total
float input = 0; // the average


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

//*******CHANGE THIS*********
// Enter the MAC address for the ethernet shield
byte mac[] = { 0x90, 0xa4, 0xda, 0x00, 0xa5, 0x59 };

//*******CHANGE THIS*********
IPAddress server(11, 111, 11, 11); // IP of host site subdomain.domain.com (ex. 11.111.11.11)

// Initialize the Ethernet client library
EthernetClient client;

//Run once at startup
void setup(void) {

  pinMode(loopSpeedLED, OUTPUT);
  pinMode(fv1HeatPin, OUTPUT);
  pinMode(fv2HeatPin, OUTPUT);
  pinMode(chillerPin, OUTPUT);
  pinMode(ambientTempProbe, INPUT);
  pinMode(coolantTempProbe, INPUT);
  pinMode(fv1TempProbe, INPUT);
  pinMode(fv2TempProbe, INPUT);
  pinMode(cSetTempPot, INPUT);
  pinMode(fv1SetTempPot, INPUT);
  pinMode(fv2SetTempPot, INPUT);
  //pinMode(fv1CrashSwitch, INPUT);
  //pinMode(fv2CrashSwitch, INPUT);
  digitalWrite(fv1HeatPin, HIGH); //initializes outputs in OFF state
  digitalWrite(fv2HeatPin, HIGH);
  digitalWrite(fv1ValveO, LOW);
  digitalWrite(fv1ValveC, LOW);
  digitalWrite(fv2ValveO, LOW);
  digitalWrite(fv2ValveC, LOW);
  digitalWrite(coolantPump, HIGH); //initializes off
  ambientTempSensor.begin();
  coolantTempSensor.begin();
  fv1TempSensor.begin();
  fv2TempSensor.begin();

  windowStartTime = millis();
  unsigned long time = millis();

  Serial.begin(9600);

  //tell the PID to range between 0 and the full window size
  fv1PID.SetOutputLimits(0, WindowSize);
  fv2PID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  fv1PID.SetMode(AUTOMATIC);
  fv2PID.SetMode(AUTOMATIC);

  analogReference(DEFAULT);

} //End SETUP

void loop(void) {

  loopSpeed();

  if (millis() - updateTimer > 200) { //update temps/setpoints every 200ms
    updateTimer = millis();
    getAmbientTemp();
    getCoolantTemp();
    getFV1Temp();
    getFV1Setpoint();
    getFV2Temp();
    getFV2Setpoint();
  }

  if (millis() - stateTimer > 1000) { //update states every second
    stateTimer = millis();
    doChillerTempControl();
    updateFV1State();
    updateFV2State();
  }

}//End LOOP


//Loop speed monitor
void loopSpeed() {
  if (digitalRead(loopSpeedLED)) {
    digitalWrite(loopSpeedLED, LOW); // turn the LED off
  }
  else {
    digitalWrite(loopSpeedLED, HIGH); // turn the LED on
  }
}

void updateFV1State() {
  enum states {
    FV1COOL,
    FV1HEAT,
    FV1IDLE,
  };
  if (aTemp > fv1SetTemp + fvHysterisis) {
    fv1State = FV1COOL;
  }
  else if (aTemp > fv1SetTemp - fvHysterisis) {
    fv1State = FV1HEAT;
  }
  else {
    fv1State = FV1IDLE;
  }

  switch (fv1State) {

    case FV1COOL:
      doFV1TempControl();
      Serial.println("FV1 COOLING");
      break;

    case FV1HEAT:
      doFV1PID;
      Serial.println("FV1 HEATING");
      break;

    case FV1IDLE:
      Serial.println("FV1 IDLE");
      break;

  }
}

void updateFV2State() {
  enum states {
    FV2COOL,
    FV2HEAT,
    FV2IDLE,
  };
  if (aTemp > fv2SetTemp + fvHysterisis) {
    fv2State = FV2COOL;
  }
  else if (aTemp > fv2SetTemp - fvHysterisis) {
    fv2State = FV2HEAT;
  }
  else {
    fv2State = FV2IDLE;
  }

  switch (fv2State) {

    case FV2COOL:
      doFV2TempControl();
      Serial.println("FV2 COOLING");
      break;

    case FV2HEAT:
      doFV2PID;
      Serial.println("FV2 HEATING");
      break;

    case FV2IDLE:
      Serial.println("FV2 IDLE");
      break;

  }
}

void doFV1TempControl() {
  if (aTemp > fv1SetTemp) { //activate when ambient temp is higher than setpoint temp
    if (fv1Temp > fv1SetTemp + fvHysterisis) {
      openFV1Valve();
      Serial.println("FV1 Valve Open");
    }
    else if (fv1Temp < fv1SetTemp) {
      closeFV1Valve();
      Serial.println("FV1 Valve Closed");
    }
    else {
      Serial.println("FV1 Idle");
    }
  }
}

void doFV2TempControl() {
  if (aTemp > fv2SetTemp) { //activate when ambient temp is higher than setpoint temp
    if (fv2Temp > fv2SetTemp + fvHysterisis) {
      openFV2Valve();
      Serial.println("FV2 Valve Open");
    }
    else if (fv2Temp < fv2SetTemp) {
      closeFV2Valve();
      Serial.println("FV2 Valve Closed");
    }
    else {
      Serial.println("FV2 Idle");
    }
  }
}

void doChillerTempControl() {
  unsigned long  time = millis();
  unsigned long previousTime;
  if (time - previousTime >= chillerOffInterval) {
    if (cTemp > cSetTemp + chillerHysterisis) {
      digitalWrite(chillerPin, LOW); //turn on chiller
      Serial.println("Chiller Cooling");
      previousTime = millis();
    }
    else if (cTemp < cSetTemp - chillerHysterisis) {
      digitalWrite(chillerPin, HIGH);
      Serial.println("Chiller Resting");
    }
    else {
      Serial.println("Chiller Resting");
    }
  }
}

float getAmbientTemp() {
  ambientTempSensor.requestTemperatures();
  float aTempSense = ambientTempSensor.getTempFByIndex(0);
  float aTemp = aTempSense;
  Serial.println(aTemp);
  return aTemp;
}

float getCoolantTemp() {
  coolantTempSensor.requestTemperatures();
  float coolantTempSense = coolantTempSensor.getTempFByIndex(0);
  float cTemp = coolantTempSense;
  Serial.println(cTemp);
  return cTemp;
}

float getFV1Temp() {
  fv1TempSensor.requestTemperatures();
  float fv1TempSense = fv1TempSensor.getTempFByIndex(0);
  if (fv1TempSense < -126) {
    Serial.print("Sensor Disconnected");
  }
  else {
    float fv1Temp = fv1TempSense;
    Serial.println(fv1Temp);
    return fv1Temp;
  }
}

float getFV2Temp() {
  fv2TempSensor.requestTemperatures();
  float fv2TempSense = fv2TempSensor.getTempFByIndex(0);
  if (fv2TempSense < -126) {
    Serial.print("Sensor Disconnected");
  }
  else {
    float fv2Temp = fv2TempSense;
    Serial.println(fv2Temp);
    return fv2Temp;
  }
}

float getFV1Setpoint() {
  fv1SensorValue = analogRead(fv1SetTempPot);
  float Setpoint1 = map(fv1SensorValue, 0, 1023, 50, 80);
  float fv1SetTemp = Setpoint1;
  total = total - readings[index];
  float fv1Temp = getFV1Temp();
  readings[index] = fv1Temp;
  total = total + readings[index];
  index = index + 1;
  if (index >= numReadings) // end of the array...
    index = 0;  // ...wrap around to the beginning:
  Input1 = total / numReadings; // calculate the average:
  return Input1;
}

float getFV2Setpoint() {
  fv2SensorValue = analogRead(fv2SetTempPot);
  float Setpoint2 = map(fv2SensorValue, 0, 1023, 30, 80);
  float fv2SetTemp = Setpoint2;
  total = total - readings[index];
  float fv2Temp = getFV2Temp();
  readings[index] = fv2Temp;
  total = total + readings[index];
  index = index + 1;
  if (index >= numReadings) // end of the array...
    index = 0;  // ...wrap around to the beginning:
  Input2 = total / numReadings; // calculate the average:
  return Input2;
}

void coolantPumpOn() {
  if (fv1State == FV1COOL || fv2State == FV2COOL) {
    digitalWrite(coolantPump, HIGH);
  }
  else {
    coolantPumpOff();
  }
}

void coolantPumpOff() {
  digitalWrite(coolantPump, LOW);
}

//Open FV1 Valve on call
void openFV1Valve() {
  unsigned long  time = millis();
  unsigned long previousTime;
  if (time - previousTime <= valveOnInterval) {
    previousTime = time;
    digitalWrite(fv1ValveC, LOW); //make sure the close pin is off
    digitalWrite(fv1ValveO, HIGH); //turn fv1 valve on
  }
  else {
    digitalWrite(fv1ValveC, LOW); //make sure the close pin is off
    digitalWrite(fv1ValveO, LOW); //turn fv1 valve on
  }
}

//Close FV1 Valve on call
void closeFV1Valve() {
  unsigned long  time = millis();
  unsigned long previousTime;
  if (time - previousTime <= valveOnInterval) {
    previousTime = time;
    digitalWrite(fv1ValveO, LOW); //turn fv1 valve on
    digitalWrite(fv1ValveC, HIGH); //make sure the close pin is off
  }
  else {
    digitalWrite(fv1ValveC, LOW); //make sure the close pin is off
    digitalWrite(fv1ValveO, LOW); //turn fv1 valve off
  }
}


//Open FV2 Valve on call
void openFV2Valve() {
  unsigned long  time = millis();
  unsigned long previousTime;
  if (time - previousTime <= valveOnInterval) {
    previousTime = time;
    digitalWrite(fv2ValveC, LOW); //make sure the close pin is off
    digitalWrite(fv2ValveO, HIGH); //turn fv1 valve on
  }
  else {
    digitalWrite(fv2ValveC, LOW); //make sure the close pin is off
    digitalWrite(fv2ValveO, LOW); //turn fv1 valve on
  }
}


//Close FV2 Valve on call
void closeFV2Valve() {
  unsigned long  time = millis();
  unsigned long previousTime;
  if (time - previousTime <= valveOnInterval) {
    previousTime = time;
    digitalWrite(fv2ValveO, LOW); //turn fv1 valve on
    digitalWrite(fv2ValveC, HIGH); //make sure the close pin is off
  }
  else {
    digitalWrite(fv2ValveC, LOW); //make sure the close pin is off
    digitalWrite(fv2ValveO, LOW); //turn fv1 valve off
  }
}


void doFV1PID() {
  fv1PID.Compute();  // send it as ASCII digits
  // turn the output pin on/off based on pid output
  if (aTemp < fv1Temp) { //heater should only heat when ambient is colder than fv temps
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (Output1 * 1000 > millis() - windowStartTime)
    {
      digitalWrite(fv1HeatPin, LOW); //turn on SSR
      Serial.println("HEATING");
    }
    else
    {
      digitalWrite(fv1HeatPin, HIGH); //turn off SSR
      Serial.println("RESTING");
    }
  }
  else
  {
    digitalWrite(fv1HeatPin, HIGH); //turn off SSR
    //setBacklight(0 , 255 , 0); //GREEN
    Serial.println("RESTING");
  }
}

void doFV2PID() {
  fv2PID.Compute();  // send it as ASCII digits
  // turn the output pin on/off based on pid output
  if (aTemp < fv2Temp) { //heater should only heat when ambient is colder than fv temps
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (Output2 * 1000 > millis() - windowStartTime)
    {
      digitalWrite(fv2HeatPin, LOW); //turn on SSR
      Serial.println("HEATING");
    }
    else
    {
      digitalWrite(fv2HeatPin, HIGH); //turn off SSR
      Serial.println("RESTING");
    }
  }
  else
  {
    digitalWrite(fv1HeatPin, HIGH); //turn off SSR
    //setBacklight(0 , 255 , 0); //GREEN
    Serial.println("RESTING");
  }
}

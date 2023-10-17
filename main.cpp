#include <Arduino.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <timing.h>
#include <pid.h>



// temp sensor vars
uint8_t tempSensorPin {A1};
uint16_t tempValueRaw {0};
float tempValueFiltered {0.0};

// motor controller vars
enum FanAction {stop, spinForward, spinBackward};
FanAction fanAction {FanAction::spinForward};
uint8_t fanActionPin0 {2};
uint8_t fanActionPin1 {3};
uint8_t fanPwmPin{9};
float fanCutOnLimit {70.0};
float fanSpeedPercent {100.0};
char param {'_'};

// controller
bool streamData {true};
PID pid {PID(
    TON(millis, 2000), 
    65.0,   // setpoint
    0.0,    // min
    100.0,  // max 
    -0.01,  // k
    10.0,   // kp
    0.01,   // ki
    1.0,    // kd
    2.0,     // deadband
    PID::DeadbandMode::ACTIVE_ON_SETPOINT
  )};


/**
 * Saves the process parameters
*/
void putParameters()
{
  int address = 0;
  EEPROM.put(address, pid.k);  address += sizeof(pid.k);
  EEPROM.put(address, pid.kp); address += sizeof(pid.kp);
  EEPROM.put(address, pid.ki); address += sizeof(pid.ki);
  EEPROM.put(address, pid.kd); address += sizeof(pid.kd);
  EEPROM.put(address, pid.setpoint); address += sizeof(pid.setpoint);
  EEPROM.put(address, pid.deadband); address += sizeof(pid.deadband);
  EEPROM.put(address, fanCutOnLimit); address += sizeof(fanCutOnLimit);
}


/**
 * Reads the process parameters
*/
void getParameters()
{
  int address = 0;
  pid.k =  EEPROM.get(address, pid.k);  address += sizeof(pid.k);
  pid.kp = EEPROM.get(address, pid.kp); address += sizeof(pid.kp);
  pid.ki = EEPROM.get(address, pid.ki); address += sizeof(pid.ki);
  pid.kd = EEPROM.get(address, pid.kd); address += sizeof(pid.kd);
  pid.setpoint = EEPROM.get(address, pid.setpoint); address += sizeof(pid.setpoint);
  pid.deadband = EEPROM.get(address, pid.deadband); address += sizeof(pid.deadband);
  fanCutOnLimit = EEPROM.get(address, fanCutOnLimit); address += sizeof(fanCutOnLimit);
}


void setup() {
  // setup timer one
  Timer1.initialize(40); // 25 khz

  // setup coms
  Serial.begin(9600);

  // setup temp sensor
  pinMode(tempSensorPin, INPUT);
  pinMode(fanActionPin0, OUTPUT);
  pinMode(fanActionPin1, OUTPUT);
  pinMode(fanPwmPin, OUTPUT);

  // write the parameters to memory
  // putParameters();  
  // read in params from memory
  getParameters();
}


void loop() {

  // ----------------- Inputs -----------------
  tempValueRaw = analogRead(tempSensorPin);
  tempValueFiltered = (-0.50)*static_cast<float>(tempValueRaw) + 177.5;

  if (Serial.available()) {
    bool printParameters = false;
    param = Serial.read();
    switch (param) {
      case 'h': // help
        Serial.println("----------- start help ------------");
        Serial.println("Options for cooling controller are:");
        Serial.println("s<float> sets the target temperature.");
        Serial.println("k<float> sets the pid system gain.");
        Serial.println("p<float> sets the pid proportional gain.");
        Serial.println("i<float> sets the pid integral gain.");
        Serial.println("d<float> sets the pid derivative gain.");
        Serial.println("c<float> sets the fan cut on/off limit.");
        Serial.println("t toggles data streaming from the pid.s");
        Serial.println("o prints the live parameters");
        Serial.println("r reads the parameters from memory.");
        Serial.println("w writes the parameters to memory.");
        Serial.println("----------- end help ------------");
        break;

      case 's':  // setpoint
        pid.setpoint = Serial.parseFloat();
        Serial.print("pid.setpoint = "); Serial.println(pid.setpoint);
        break;
      
      case 'k':  // system gain
        pid.k = Serial.parseFloat();
        Serial.print("pid.k = "); Serial.println(pid.k);
        break;

      case 'p':  // proportional gain
        pid.kp = Serial.parseFloat();
        Serial.print("pid.kp = "); Serial.println(pid.kp);
        break;

      case 'i':  // integral gain
        pid.ki = Serial.parseFloat();
        Serial.print("pid.ki = "); Serial.println(pid.ki);
        break;

      case 'd':  // derivative gain
        pid.kd = Serial.parseFloat();
        Serial.print("pid.kd = "); Serial.println(pid.kd);
        break;

      case 'c':  // fan cut on/off limit
        fanCutOnLimit = Serial.parseFloat();
        Serial.print("fan on/off limit = "); Serial.println(fanCutOnLimit);
        break;

      case 't':  // toggles the streaming of the values
        streamData = !streamData;
        Serial.print("streamData = "); Serial.println(streamData);
        break;

      case 'o': // output the parameters from the eeprom
        printParameters = true;
        break;

      case 'r': // read parameters from eeprom and prints them
        getParameters(); 
        printParameters = true;
        break;

      case 'w':  // save parameters to eeprom
        putParameters();
        printParameters = true;
        break;
    }

    if (printParameters) {
      Serial.print("pid.k = ");  Serial.println(pid.k);
      Serial.print("pid.kp = "); Serial.println(pid.kp);
      Serial.print("pid.ki = "); Serial.println(pid.ki); 
      Serial.print("pid.kd = "); Serial.println(pid.kd); 
      Serial.print("pid.setpoint = "); Serial.println(pid.setpoint); 
      Serial.print("pid.deadband = "); Serial.println(pid.deadband); 
      Serial.print("fan on/off limit = "); Serial.println(fanCutOnLimit);
    }
  }
  

  // ----------------- Processing -----------------
  fanSpeedPercent = pid.call(tempValueFiltered).controlValue;


  // ----------------- Outputs -----------------
  // control fan state
  if (fanAction == FanAction::spinForward) {
    digitalWrite(fanActionPin0, HIGH);
    digitalWrite(fanActionPin1, LOW);
  }

  // control fan speed based on state
  if (fanAction == FanAction::stop || fanSpeedPercent < fanCutOnLimit) {
    // set pwm out to zero rather than "locking" the fan
    Timer1.pwm(fanPwmPin, 0);
  } else {
    Timer1.pwm(fanPwmPin, 1023*(fanSpeedPercent/100));
  }

  // ----------------- Diagnostics -----------------

  // print on the interval specified by print timer
  if (pid.timer.DN and streamData) {
    Serial.print(">PV:");  Serial.println(tempValueFiltered);
    Serial.print(">SP:");  Serial.println(pid.setpoint);
    Serial.print(">dbA:"); Serial.println(pid.deadbandActive);
    Serial.print(">cp:");  Serial.println(pid.cp);
    Serial.print(">ci:");  Serial.println(pid.ci);
    Serial.print(">cd:");  Serial.println(pid.cd);
    Serial.print(">CV:");  Serial.println(pid.controlValue);
  }
}
  
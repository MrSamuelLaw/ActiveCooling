#include <Arduino.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <timing.h>
#include <pid.h>



// temp sensor vars
uint8_t tempSensorPin {A1};
uint16_t tempValueRaw {0};
float alpha {0.50};
float tempValueConverted {0.0};
float tempValueFiltered {0.0};
TON tempPollTimer {TON(millis, 500)};

// motor controller vars
uint8_t fan1ActionPin0 {2};
uint8_t fan1ActionPin1 {3};
uint8_t fan2ActionPin0 {4};
uint8_t fan2ActionPin1 {5};
uint8_t fan1PwmPin{9};
uint8_t fan2PwmPin{10};
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
  EEPROM.put(address, alpha); address += sizeof(alpha);
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
  alpha = EEPROM.get(address, alpha); address += sizeof(alpha);
}


void setup() {
  // setup timer one
  Timer1.initialize(40); // 25 khz

  // setup coms
  Serial.begin(9600);

  // setup temp sensor
  pinMode(tempSensorPin, INPUT);
  pinMode(fan1ActionPin0, OUTPUT);
  pinMode(fan1ActionPin1, OUTPUT);
  pinMode(fan2ActionPin0, OUTPUT);
  pinMode(fan2ActionPin1, OUTPUT);
  pinMode(fan1PwmPin, OUTPUT);
  pinMode(fan2PwmPin, OUTPUT);

  // set the fan direction to forward for both fans
  digitalWrite(fan1ActionPin0, HIGH);
  digitalWrite(fan1ActionPin1, LOW);
  digitalWrite(fan2ActionPin0, LOW);
  digitalWrite(fan2ActionPin1, HIGH);

  // write the parameters to memory
  // putParameters();  
  // read in params from memory
  getParameters();
}


void loop() {

  // ----------------- Inputs -----------------
  tempValueRaw = analogRead(tempSensorPin);
  tempValueConverted = (-0.50)*static_cast<float>(tempValueRaw) + 177.5;

  // low pass filter to steady the temp readings
  if (tempPollTimer.call(!tempPollTimer.DN).DN) {
    tempValueFiltered = (tempValueFiltered * alpha) + ( (1.0 - alpha) * tempValueConverted );
    if (isnan(tempValueFiltered)) {
      tempValueFiltered = tempValueConverted;
    }
  }

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
        Serial.println("a<float> sets the temperature low pass gain.");
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

      case 'a':  // temp sensor low pass gain
        alpha = Serial.parseFloat();
        Serial.print("low pass gain = "); Serial.println(alpha);
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
      Serial.print("temperature low pass gain = "); Serial.println(alpha);
    }
  }
  

  // ----------------- Processing -----------------
  fanSpeedPercent = pid.call(tempValueFiltered).controlValue;


  // ----------------- Outputs -----------------

  // control fan speed based on state
  if (fanSpeedPercent < fanCutOnLimit) {
    // set pwm out to zero rather than "locking" the fan
    Timer1.pwm(fan1PwmPin, 0);
    Timer1.pwm(fan2PwmPin, 0);
  } else {
    Timer1.pwm(fan1PwmPin, 1023*(fanSpeedPercent/100));
    Timer1.pwm(fan2PwmPin, 1023*(fanSpeedPercent/100));
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
  
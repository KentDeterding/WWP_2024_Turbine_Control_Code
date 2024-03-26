#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>
#include "filter.h"
#include "commands.h"


#define LA_ID_NUM 0         // ID number of the linear actuator
#define PCC_Relay_Pin 33    // Relay control pin


// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dacValue = 4090; // 0 - 4095
float targetResistance = 8;

// Timers
unsigned long printTimer;
unsigned long printTimerInterval = 1000;
unsigned long resistanceTracingTimer;
unsigned long resistanceTrackingInterval = 100;

// RPM
#define RPM_Pin 29
struct Filter* rpm_filter = CreateFilter(10, 14);

// Global State
bool trackResistance = false;
int dacStepSize = 100;
bool printOutput = true;


void setup () {
    Serial.begin(9600);
    delay(1000); // Wait so serial monitor can be opened
    Serial.println("Starting up...");
    while (!Serial.available()) {
        delay(10);
    }
    bool success = true;

    //Linear Actuator
    myServo.begin(32);
    delay(100);
    if (myServo.available()) {
        Serial.println("Linear actuator ready");
    } else {
        Serial.println("Linear actuator error");
        success = false;
    }
    myServo.movingSpeed(LA_ID_NUM, 750);

    //INA260
    ina260.begin(0x40);
    delay(10);
    if (ina260.conversionReady()) {
        Serial.println("INA260 ready");
    } else {
        Serial.println("INA260 error");
        success = false;
    }
    
    //DAC
    dac.begin(0x64);
    delay(10);
    dac.setVoltage(dacValue, false);
    Serial.println("DAC ready");

    //Relay
    pinMode(PCC_Relay_Pin, OUTPUT);

    // Start RPM Tracking
    attachInterrupt(digitalPinToInterrupt(RPM_Pin), RPM_Interrupt, RISING);

    if (success) {
        Serial.println("Setup complete");
    } else {
        Serial.println("Setup failed");
    }

    printTimer = millis();
    resistanceTracingTimer = millis();
}

void loop () {
    if (Serial.available() > 0) {
        String serialInput = Serial.readStringUntil('\n');
        ProcessCommand(serialInput);
        Serial.flush();
    }

    if (printTimer < millis() && printOutput) {
        printTimer += printTimerInterval;
        PrintOutput();
    }

    // Track load resistance
    if (resistanceTracingTimer < millis() && trackResistance) {
        resistanceTracingTimer += resistanceTrackingInterval;

        float voltage = ina260.readBusVoltage();
        float current = ina260.readCurrent();
        float resistance = voltage / current;

        float difference = resistance - targetResistance;

        if (difference > 0) {
            dacValue += dacStepSize;
        } else {
            dacValue -= dacStepSize;
        }
        dac.setVoltage(dacValue, false);
    }
}

String PadString (String str) {
    while (str.length() < 8) {
        str = " " + str;
    }
    return str;
}

void PrintOutput () {
    Serial.println("");
    Serial.println("");
    Serial.println("Time: \t\t" + PadString(String(millis())));
    Serial.println("Dac: \t\t" + PadString(String(dacValue)));
    Serial.println("Power: \t\t" + PadString(String(ina260.readPower())));
    Serial.println("Voltage: \t" + PadString(String(ina260.readBusVoltage())));
    Serial.println("LA Position: \t" + String(myServo.presentPosition(LA_ID_NUM)));
    String relayState = digitalRead(PCC_Relay_Pin) ? "High" : "Low";
    Serial.println("Relay State: " + relayState);
    Serial.println("RPM: " + String(GetRpmBuffered(rpm_filter)));
}


void ProcessCommand (String serialInput) {
    String command = NextArg(&serialInput);

    switch (MatchCommand(command)) {
        case Command::INVALID:
            Serial.println("Invalid command: try \"help\"");
            break;
        case Command::HELP:
            Help();
            break;
        case Command::SET:
            Set(command);
            break;
        case Command::SWITCH:
            Switch(command);
            break;
        default:
            Serial.println("Command not implemented");
    }
}

// 
void Set(String command) {
    String arg = NextArg(&command).toLowerCase();
    
    if (arg == "dac") {
        dacValue = NextArg(&command).toInt();
        dac.setVoltage(dacValue, false);
        Serial.println("DAC set to " + String(dacValue));
    } else if (arg == "la") {
        int pos = NextArg(&command).toInt();
        myServo.goalPosition(LA_ID_NUM, pos);
        Serial.println("Linear Actuator set to " + String(pos));
    } else if (arg == "res") {
        targetResistance = NextArg(&command).toFloat();
    } else {
        Serial.println("Invalid subcommand for set");
        Serial.println("Try \"help\"");
    }
}

//
void Switch(String command) {
    String arg = NextArg(&command).toLowerCase();

    if (arg == "pcc") {
        digitalWrite(PCC_Relay_Pin, !digitalRead(PCC_Relay_Pin));
        Serial.println("Relay set to " + digitalRead(PCC_Relay_Pin) ? "High" : "Low");
    } else if (arg = "res") {

    } else {
        Serial.println("Invalid subcommand for switch");
        Serial.println("Try \"help\"");
    }
}

// Interrupt for measuring the RPM
void RPM_Interrupt () {
    int time = (int)micros();
    Insert(rpm_filter, time);
}
#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>
#include "filter.h"
#include "commands.h"

#define LA_ID_NUM 0         // ID number of the linear actuator
#define PCC_Relay_Pin 33    // Relay control pin
#define Safety_Switch_Pin 11


// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dacValue = 50; // 0 - 4095
float targetResistance = 8;

// RPM
#define RPM_Pin 29
struct Filter* rpm_filter = CreateFilter(10, 8);

// Timers
unsigned long printTimer;
unsigned long printTimerInterval = 30; // was 1000, but I want to see the limit
unsigned long resistanceTrackingTimer;
unsigned long resistanceTrackingInterval = 100;
unsigned long mpptTimer;
unsigned long mpptInterval = 250;
unsigned long sweepTimer;
unsigned long sweepInterval = 500;

// Global State
bool trackResistance = false;
int dacStepSize = 20;
bool printOutput = true;
bool mpptEnabled = false;
float last_power = 0;
bool sweepDac = false;

void setup () {
    Serial.begin(9600);
    while (!Serial) { // Wait so serial monitor can be opened
        delay(10);
    }
    Serial.println("Starting up...");
    bool success = true;

    // Saftey Switch
    pinMode(Safety_Switch_Pin, INPUT);

    //Linear Actuator
    myServo.begin(32);
    delay(100);
    if (myServo.available()) {
        myServo.movingSpeed(LA_ID_NUM, 750);
        Serial.println("Linear actuator ready");
    } else {
        Serial.println("Linear actuator error");
        success = false;
    }

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

    Serial.println("Type \"help\" for a list of commands");

    printTimer = millis();
    resistanceTrackingTimer = millis();
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

/*
    if (digitalRead(Safety_Switch_Pin) != HIGH) {
        myServo.goalPosition(LA_ID_NUM, 0);
        dacValue = 4095;
        dac.setVoltage(dacValue, false);
    }
*/

    // Track load resistance
    if (resistanceTrackingTimer < millis() && trackResistance) {
        resistanceTrackingTimer += resistanceTrackingInterval;

        // Dac val -> Load Current is a linear relationship
        // Current(mA) = m*dacValue + b
        const float m = 1.59404;
        const float b = 2.53791;

        float voltage = ina260.readBusVoltage();
        float current = ina260.readCurrent();
        float resistance = voltage / current;

        float difference = resistance - targetResistance;

        if (difference > 0) {
            dacValue += dacStepSize;
        } else {
            dacValue -= dacStepSize;
        }

        if (dacValue > 4095) {
            dacValue = 4095;
        } else if (dacValue < 0) {
            dacValue = 0;
        }
        dac.setVoltage(dacValue, false);

        //float targetCurrent = voltage / targetResistance;
        //dacValue = (targetCurrent - b) / m;
        //dac.setVoltage(dacValue, false);
    }

    if (mpptTimer < millis() && mpptEnabled) {
        mpptTimer += mpptInterval;

        float voltage = ina260.readBusVoltage();
        float current = ina260.readCurrent();
        float power = voltage * current;

        if (power > last_power) {
            dacValue += dacStepSize / 2;
        } else {
            dacValue -= dacStepSize / 2;
        }

        last_power = power;
        dac.setVoltage(dacValue, false);
    }

    // Just a testing fuction to get some data
    if (sweepTimer < millis() && sweepDac) {
        sweepTimer += sweepInterval;

        dacValue += 20;
        if (dacValue > 4095) {
            dacValue = 4095;
            sweepDac = false;
            trackResistance = true;
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
    String relayState = digitalRead(PCC_Relay_Pin) ? "High" : "Low";
    String turbineVoltage = digitalRead(30) ? "off" : "on";
    String relayStateStr = PadString(relayState);
    String safetySwitchStr = PadString(digitalRead(Safety_Switch_Pin) ? "open" : "closed"); // Should shutdown when closed
    String turbineVoltageStr = PadString(turbineVoltage);
    String laPosStr = PadString(String(myServo.presentPosition(LA_ID_NUM)));
    String mpptStatus = PadString(mpptEnabled ? "true" : "false");
    String dacValStr = PadString(String(dacValue));
    float current = ina260.readCurrent();
    float voltage = ina260.readBusVoltage();
    String resistanceStr = PadString(String(voltage / current));
    String currentStr = PadString(String(current));
    String voltStr = PadString(String(voltage));
    String powerStr = PadString(String(ina260.readPower()));
    String rpmStr = PadString(String(GetRpmBuffered(rpm_filter)));
    Serial.print("\n\n\n");
    Serial.println("Time:                " + PadString(String(millis())));
    Serial.println("\tRelay State: " + relayStateStr);
    Serial.println("\tSafety:      " + safetySwitchStr);
    Serial.println("\tT-Status:    " + turbineVoltageStr);
    Serial.println("\tLA Position: " + laPosStr);
    Serial.println("\tMPPT Enabled:" + mpptStatus);
    if (trackResistance) {
        Serial.println("\tTarget Res:  " + PadString(String(targetResistance)));
    } else {
        Serial.println("\tTarget Res:       N/A");
    }
    Serial.println("\tDac:         " + dacValStr);
    Serial.println("\tResistance:  " + resistanceStr);
    Serial.println("\tCurrent:     " + currentStr);
    Serial.println("\tVoltage:     " + voltStr);
    Serial.println("\tPower:       " + powerStr);
    Serial.println("\tRPM:         " + rpmStr);
}

void ProcessCommand (String &serialInput) {
    String command = serialInput;
    String cmd = NextArg(command);

    switch (MatchCommand(cmd)) {
        case Command::INVALID:
            Serial.println("Invalid command: try \"help\"");
            break;
        case Command::HELP:
            Serial.println(Help());
            break;
        case Command::SET:
            Set(command);
            break;
        case Command::TOGGLE:
            Toggle(command);
            break;
        default:
            Serial.println("Command not implemented");
    }
}

void Set(String &command) {
    String arg = NextArg(command).toLowerCase();
    
    if (arg == "dac") {
        dacValue = NextArg(command).toInt();
        dac.setVoltage(dacValue, false);
        Serial.println("DAC set to " + String(dacValue));
    } else if (arg == "la") {
        int pos = NextArg(command).toInt();
        myServo.goalPosition(LA_ID_NUM, pos);
        Serial.println("Linear Actuator set to " + String(pos));
    } else if (arg == "res") {
        targetResistance = NextArg(command).toFloat();
    } else {
        Serial.println("Invalid subcommand for set");
        Serial.println("Try \"help\"");
    }
}

void Toggle(String &command) {
    String arg = NextArg(command).toLowerCase();

    if (arg == "pcc") {
        digitalWrite(PCC_Relay_Pin, !digitalRead(PCC_Relay_Pin));
    } else if (arg = "res") {
        trackResistance = !trackResistance;
        resistanceTrackingTimer = millis();
    } else if (arg = "mppt") {
        trackResistance = false;
        mpptEnabled = !mpptEnabled;
        mpptTimer = millis();
    } else if (arg = "print") {
        printOutput = !printOutput;
    } else if (arg = "sweep") {
        trackResistance = false;
        mpptEnabled = false;
        sweepDac = true;
        sweepTimer = millis();
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
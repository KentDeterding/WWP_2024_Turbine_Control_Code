#include <Arduino.h>
#line 1 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>
#include "types.h"


#define LA_ID_NUM 0         // ID number of the linear actuator
#define PCC_Relay_Pin 33


// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dacValue = 4090; // 0 - 4095

// Timers
unsigned long printTimer = 0;
unsigned long printTimerInterval = 1000;


#line 27 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
void setup();
#line 70 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
void loop();
#line 83 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
String PadString(String str);
#line 90 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
void PrintOutput();
#line 99 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
Command getCommand(String command);
#line 113 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
void ProcessCommand(String serialInput);
#line 27 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
void setup () {
    Serial.begin(9600);
    delay(1000); // Wait so serial monitor can be opened
    Serial.println("Starting up...");
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

    if (success) {
        Serial.println("Setup complete");
    } else {
        Serial.println("Setup failed");
    }
}

void loop () {
    if (Serial.available() > 0) {
        String serialInput = Serial.readStringUntil('\n');
        ProcessCommand(serialInput);
        Serial.flush();
    }

    if (printTimer < millis()) {
        printTimer += printTimerInterval;
        PrintOutput();
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
    Serial.println("Time: \t\t" + PadString(String(millis())));
    Serial.println("Dac: \t\t" + PadString(String(dacValue)));
    Serial.println("Power: \t\t" + PadString(String(ina260.readPower())));
    Serial.println("Voltage: \t" + PadString(String(ina260.readBusVoltage())));
    Serial.println("LA Position: \t" + PadString(String(myServo.presentPosition(LA_ID_NUM))));
}

Command getCommand (String command) {
    Serial.println("Comman:" + command);

    if (command == "setDac") {
        return Command::SETDAC;
    } else if (command.toLowerCase() == "setla") {
        return Command::SETLA;
    } else if (command.toLowerCase() == "switchpcc") {
        return Command::SWITCHPCC;
    } else {
        return Command::INVALID;
    }
}

void ProcessCommand (String serialInput) {
    Command command = getCommand(serialInput.substring(0, serialInput.indexOf(" ")));
    String args = serialInput.substring(serialInput.indexOf(" ") + 1);

    switch (command) {
        case Command::INVALID:
            Serial.println("Invalid command");
            break;
        case Command::SETDAC:
            dacValue = args.toInt();
            dac.setVoltage(dacValue, false);
            Serial.println("DAC set to " + String(dacValue));
            break;
        case Command::SETLA:
            myServo.goalPosition(LA_ID_NUM, args.toInt());
            Serial.println("Linear Actuator set to " + String(args.toInt()));
            break;
        case Command::SWITCHPCC:
            digitalWrite(PCC_Relay_Pin, !digitalRead(PCC_Relay_Pin));
    }
}

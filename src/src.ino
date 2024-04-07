#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>
#include "filter.h"
#include "commands.h"

#define LA_ID_NUM 0             // ID number of the linear actuator
#define PCC_RELAY_PIN 33        // Relay control pin
#define SAFETY_SWITCH_PIN 11    // Pin number for the safety switch input
#define RPM_PIN 29              //


// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dac_value = 50; // 0 - 4095
float targetResistance = 8;

// RPM
struct Filter* rpm_filter = new_filter(10, 8);

// Timers
unsigned long print_timer;
unsigned long print_timer_interval = 200;
unsigned long resistance_tracking_timer;
unsigned long resistance_tracking_interval = 100;
unsigned long mppt_timer;
unsigned long mppt_interval = 250;
unsigned long sweep_timer;
unsigned long sweep_interval = 500;

// Global State
bool track_resistance = false;
int dac_step_size = 20;
bool print_output = true;
bool mppt_enabled = false;
float last_power = 0;
bool sweep_dac = false;

void setup () {
    Serial.begin(9600);
    while (!Serial) {
        // Wait so serial monitor can be opened
    }
    Serial.println("Starting up...");

    // Saftey Switch
    pinMode(SAFETY_SWITCH_PIN, INPUT);

    //Relay
    pinMode(PCC_RELAY_PIN, OUTPUT);
    digitalWrite(PCC_RELAY_PIN, HIGH); // Start with turbine-side powered

    //Linear Actuator
    myServo.begin(32);
    myServo.movingSpeed(LA_ID_NUM, 750);

    //INA260
    ina260.begin(0x40);
    if (!ina260.conversionReady()) {
        Serial.println("INA260 error");
    }

    //DAC
    dac.begin(0x64);
    dac.setVoltage(dac_value, false);

    // Start RPM Tracking
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), RPM_Interrupt, RISING);

    Serial.println("Setup finished");
    Serial.println("Type \"help\" for a list of commands");

    // Init timers
    print_timer = millis();
    resistance_tracking_timer = millis();
}

void loop () {
    if (Serial.available() > 0) {
        String serialInput = Serial.readStringUntil('\n');
        ProcessCommand(serialInput);
        Serial.flush();
    }

    if (print_timer < millis() && print_output) {
        print_timer += print_timer_interval;
        PrintOutput();
    }

    if (digitalRead(SAFETY_SWITCH_PIN) != HIGH) {
        myServo.goalPosition(LA_ID_NUM, 0);
        dac_value = 4095;
        dac.setVoltage(dac_value, false);
    }

    // Track load resistance
    if (resistance_tracking_timer < millis() && track_resistance) {
        resistance_tracking_timer += resistance_tracking_interval;

        // Dac val -> Load Current is a linear relationship
        // Current(mA) = m*dac_value + b
        const float m = 1.59404;
        const float b = 2.53791;

        float voltage = ina260.readBusVoltage();
        float current = ina260.readCurrent();
        float resistance = voltage / current;

        float difference = resistance - targetResistance;

        if (difference > 0) {
            dac_value += dac_step_size;
        } else {
            dac_value -= dac_step_size;
        }

        if (dac_value > 4095) {
            dac_value = 4095;
        } else if (dac_value < 0) {
            dac_value = 0;
        }
        dac.setVoltage(dac_value, false);

        //float targetCurrent = voltage / targetResistance;
        //dac_value = (targetCurrent - b) / m;
        //dac.setVoltage(dac_value, false);
    }

    if (mppt_timer < millis() && mppt_enabled) {
        mppt_timer += mppt_interval;

        float voltage = ina260.readBusVoltage();
        float current = ina260.readCurrent();
        float power = voltage * current;

        if (power > last_power) {
            dac_value += dac_step_size / 2;
        } else {
            dac_value -= dac_step_size / 2;
        }

        last_power = power;
        dac.setVoltage(dac_value, false);
    }

    // Just a testing fuction to get some data
    if (sweep_timer < millis() && sweep_dac) {
        sweep_timer += sweep_interval;

        dac_value += 20;
        if (dac_value > 4095) {
            dac_value = 4095;
            sweep_dac = false;
            track_resistance = true;
        }
        dac.setVoltage(dac_value, false);
    }
}

String PadString (String str) {
    while (str.length() < 8) {
        str = " " + str;
    }
    return str;
}

void PrintOutput () {
    String relayState = digitalRead(PCC_RELAY_PIN) ? "High" : "Low";
    String turbineVoltage = digitalRead(30) ? "off" : "on";
    String relayStateStr = PadString(relayState);
    String safetySwitchStr = PadString(digitalRead(SAFETY_SWITCH_PIN) ? "open" : "closed"); // Should shutdown when closed
    String turbineVoltageStr = PadString(turbineVoltage);
    String laPosStr = PadString(String(myServo.presentPosition(LA_ID_NUM)));
    String mpptStatus = PadString(mppt_enabled ? "true" : "false");
    String dacValStr = PadString(String(dac_value));
    float current = ina260.readCurrent();
    float voltage = ina260.readBusVoltage();
    String resistanceStr = PadString(String(voltage / current));
    String currentStr = PadString(String(current));
    String voltStr = PadString(String(voltage));
    String powerStr = PadString(String(ina260.readPower()));
    String rpmStr = PadString(String(get_rpm_buffered(rpm_filter)));
    Serial.print("\n\n\n");
    Serial.println("Time:                " + PadString(String(millis())));
    Serial.println("\tRelay State: " + relayStateStr);
    Serial.println("\tSafety:      " + safetySwitchStr);
    Serial.println("\tT-Status:    " + turbineVoltageStr);
    Serial.println("\tLA Position: " + laPosStr);
    Serial.println("\tMPPT Enabled:" + mpptStatus);
    if (track_resistance) {
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
    String cmd = next_arg(command);

    switch (match_command(cmd)) {
        case Command::INVALID:
            Serial.println("Invalid command: try \"help\"");
            break;
        case Command::HELP:
            Serial.println(help());
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
    String arg = next_arg(command).toLowerCase();
    
    if (arg == "dac") {
        dac_value = next_arg(command).toInt();
        dac.setVoltage(dac_value, false);
        Serial.println("DAC set to " + String(dac_value));
    } else if (arg == "la") {
        int pos = next_arg(command).toInt();
        myServo.goalPosition(LA_ID_NUM, pos);
        Serial.println("Linear Actuator set to " + String(pos));
    } else if (arg == "res") {
        targetResistance = next_arg(command).toFloat();
    } else {
        Serial.println("Invalid subcommand for set");
        Serial.println("Try \"help\"");
    }
}

void Toggle(String &command) {
    String arg = next_arg(command).toLowerCase();

    if (arg == "pcc") {
        digitalWrite(PCC_RELAY_PIN, !digitalRead(PCC_RELAY_PIN));
    } else if (arg = "res") {
        track_resistance = !track_resistance;
        resistance_tracking_timer = millis();
    } else if (arg = "mppt") {
        track_resistance = false;
        mppt_enabled = !mppt_enabled;
        mppt_timer = millis();
    } else if (arg = "print") {
        print_output = !print_output;
    } else if (arg = "sweep") {
        track_resistance = false;
        mppt_enabled = false;
        sweep_dac = true;
        sweep_timer = millis();
    } else {
        Serial.println("Invalid subcommand for switch");
        Serial.println("Try \"help\"");
    }
}

// Interrupt for measuring the RPM
void RPM_Interrupt () {
    int time = (int)micros();
    insert(rpm_filter, time);
}
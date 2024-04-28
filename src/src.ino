#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>
#include "commands.h"
#include "rpmFilter.h"
#include "digitalFilter.h"

#define FAN_PIN 20              // PWM on this pin controls the rpm of the cooling fan
#define LA_ID_NUM 0             // ID number of the linear actuator
#define PCC_STATUS_PIN 30       // Reads HIGH when the turbine side voltage is HIGH
#define PCC_RELAY_PIN 33        // Relay control pin
#define SAFETY_SWITCH_PIN 11    // Pin number for the safety switch input
#define RPM_PIN 29              // Square wave input for calculating RPM

enum Modes mode = Modes::MANUAL;
enum DacMode dac_mode = DacMode::DIRECT_DAC;
enum PitchMode pitch_mode = PitchMode::DIRECT_LA;
enum States state = States::STARTUP;

// Linear Actuator
PA12 myServo(&Serial1, 16, 1);
//PA12 linear_actuator = PA12(&Serial1, 16, 1); // possible replace alias for myServo
const int cut_in_position = 900;
const int feathered_position = 3200;

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dac_value = 50; // 0 - 4095
float targetResistance = 8;

// RPM
struct RpmFilter *rpm_filter = new_rpm_filter(6, 8);

struct {
    float prev_power = 0.0;
    int prev_la_pos = 0;
    const int la_step_size = 50;
    uint16_t prev_dac_value = 0;
    const uint16_t dac_step_size = 25;
    uint16_t prev_dir = 1;
} mppt_data;

// Timers
unsigned long print_timer;
const unsigned long print_timer_interval = 500;
unsigned long resistance_tracking_timer;
const unsigned long resistance_tracking_interval = 100;
unsigned long mppt_timer;
const unsigned long mppt_interval = 250;
unsigned long sweep_timer;
const unsigned long sweep_interval = 500;
unsigned long read_timer;
const unsigned long read_interval = 50;
unsigned long state_machine_timer;
const unsigned long state_machine_interval = 100;

// Global State
int dac_step_size = 20;
bool print_output = true;
bool mppt_enabled = false;
float last_power = 0;
bool sweep_dac = false;

struct DigitalFilter *power_filter = new_digital_filter(5);

void setup() {
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

    //DAC
    dac.begin(0x64);
    dac.setVoltage(dac_value, false);

    // Start RPM Tracking
    attachInterrupt(digitalPinToInterrupt(RPM_PIN), RPM_Interrupt, RISING);

    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, HIGH);

    digitalWrite(PCC_RELAY_PIN, LOW);

    Serial.println("Setup finished");
    Serial.println("Type \"help\" for a list of commands");

    // Init timers
    print_timer = millis();
    resistance_tracking_timer = millis();
    read_timer = millis();
}

void loop() {
    if (Serial.available() > 0) {
        String serialInput = Serial.readStringUntil('\n');
        ProcessCommand(serialInput);
        Serial.flush();
    }

    if (print_timer < millis() && print_output) {
        print_timer += print_timer_interval;
        PrintOutput();
    }

    if (read_timer < millis()) {
        read_timer += read_interval;
        digital_filter_insert(power_filter, (float)ina260.readPower());
    }

    switch (mode) {
        case Modes::MANUAL:
            if (resistance_tracking_timer < millis() && dac_mode == DacMode::RESISTANCE) {
                resistance_tracking_timer += resistance_tracking_interval;

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
            }
            break;
        case Modes::AUTO:
            if (state_machine_timer > millis()) {
                state_machine_timer += state_machine_interval;
                // break;
            }
            
            switch (state) {
                case States::SAFETY:
                    if (safety_switch_closed() && pcc_connected()) {
                        state = States::STARTUP;
                    }
                    break;
                case States::STARTUP:
                    digitalWrite(PCC_RELAY_PIN, HIGH);
                    Serial.println("Powering up t-side...");
                    delay(3000);

                    //while (!myServo.available()) {}
                    // TODO: Don't start at full force in high wind speeds
                    myServo.goalPosition(LA_ID_NUM, cut_in_position);
                    //while (myServo.Moving(LA_ID_NUM)) {} // wait for linear actuator to stop moving
                    while (abs(myServo.presentPosition(LA_ID_NUM) - myServo.goalPosition(LA_ID_NUM)) > 400) {
                        Serial.println("Delta:\t" + (myServo.presentPosition(LA_ID_NUM) - myServo.goalPosition(LA_ID_NUM)));
                    }

                    digitalWrite(PCC_RELAY_PIN, LOW);

                    dac_value = 100;
                    dac.setVoltage(dac_value, false);

                    state = States::AWAIT_POWER;

                    break;
                case States::AWAIT_POWER:
                    // TODO: add a buffer to ensure power is stable
                    if (!digitalRead(PCC_STATUS_PIN) && rpm_filter_get(rpm_filter) > 400.0) {
                        mppt_data.prev_power = digital_filter_get_avg(power_filter);
                        mppt_data.prev_dir = 1;
                        state = States::POWER_CURVE;
                    }
                    break;
                case States::POWER_CURVE:
                    digital_filter_get_avg(power_filter);
                    myServo.goalPosition(LA_ID_NUM, 700);
                    dac_value = 150;
                    dac.setVoltage(dac_value, false);

                    if (digitalRead(SAFETY_SWITCH_PIN) || !pcc_connected()) {
                        myServo.goalPosition(LA_ID_NUM, feathered_position);
                        state = States::SAFETY;
                    }

                    // TODO: Check if la is on

                    // Perturb the system

                    // Allow the system to reach new equillibrium
                    // Observe the system
                    if (digital_filter_get_avg(power_filter) > mppt_data.prev_power) {// If the perturbation caused better performance,
                        // move in the same direction
                        dac_value += mppt_data.dac_step_size * mppt_data.prev_dir;
                    } else {
                        // move in the opposite direction
                        mppt_data.prev_dir *= -1;
                        dac_value += mppt_data.dac_step_size * mppt_data.prev_dir;
                    }
                    break;
                case States::REGULATE:

                    // TODO: Keep power/rpm stable

                    break;
            }
            break;
        case Modes::TEST:

            break;
    }
}

bool safety_switch_closed() {
    return !digitalRead(SAFETY_SWITCH_PIN);
}

bool pcc_connected() {
    return !(!digitalRead(PCC_STATUS_PIN) && ina260.readBusVoltage() < 50.0 && rpm_filter_get(rpm_filter) > 200);
}

String PadString(String str) {
    while (str.length() < 8) {
        str = " " + str;
    }
    return str;
}

void PrintOutput() {
    String relayState = digitalRead(PCC_RELAY_PIN) ? "High" : "Low";
    String pcc_connected = digitalRead(PCC_RELAY_PIN) && ina260.readPower() > 1.0 ? "false" : "true";
    String turbineVoltage = digitalRead(30) ? "off" : "on";
    String relayStateStr = PadString(relayState);
    String safetySwitchStr = PadString(digitalRead(SAFETY_SWITCH_PIN) ? "open" : "closed"); // Should shutdown when closed
    String turbineVoltageStr = PadString(turbineVoltage);
    String laTargetStr = PadString(String(myServo.goalPosition(LA_ID_NUM)));
    String laPosStr = PadString(String(myServo.presentPosition(LA_ID_NUM)));
    String mpptStatus = PadString(mppt_enabled ? "true" : "false");
    String dacValStr = PadString(String(dac_value));
    float current = ina260.readCurrent();
    float voltage = ina260.readBusVoltage();
    String resistanceStr = PadString(String(voltage / current));
    String currentStr = PadString(String(current));
    String voltStr = PadString(String(voltage));
    String powerStr = PadString(String(digital_filter_get_avg(power_filter)));
    String rpmStr = PadString(String(rpm_filter_get(rpm_filter)));
    String current_state;
    if (mode != Modes::AUTO) {
        current_state = "N/A";
    } else {
        switch (state) {
            case States::SAFETY:
                current_state = "safety";
                break;
            case States::STARTUP:
                current_state = "startup";
                break;
            case States::AWAIT_POWER:
                current_state = "await power";
                break;
            case States::POWER_CURVE:
                current_state = "power curve";
                break;
            case States::REGULATE:
                current_state = "regulate";
                break;
            default:
                current_state = "nil";
                break;
        }
    }
    Serial.print("\n\n\n");
    Serial.println("Time:                " + PadString(String(millis())));
    Serial.println("\tFSM:          " + current_state);
    Serial.println("\tRelay State:  " + relayStateStr);
    Serial.println("\tPCC Connected:" + pcc_connected);
    Serial.println("\tSafety:       " + safetySwitchStr);
    Serial.println("\tT-Status:     " + turbineVoltageStr);
    Serial.println("\tLA Target:    " + laTargetStr);
    Serial.println("\tLA Position:  " + laPosStr);
    Serial.println("\tMPPT Enabled: " + mpptStatus);
    if (dac_mode == DacMode::RESISTANCE) {
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

void ProcessCommand(String &serialInput) {
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
            set(command);
            break;
        case Command::TOGGLE:
            toggle(command);
            break;
        case Command::SELECT:
            select(command);
            break;
        default:
            Serial.println("Command not implemented");
    }
}

void set(String &command) {
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

void toggle(String &command) {
    String arg = next_arg(command).toLowerCase();

    if (arg == "pcc") {
        digitalWrite(PCC_RELAY_PIN, !digitalRead(PCC_RELAY_PIN));
    } else if (arg = "res") {
        if (dac_mode == DacMode::DIRECT_DAC) {
            dac_mode = DacMode::RESISTANCE;
            resistance_tracking_timer = millis();
        } else {
            dac_mode = DacMode::DIRECT_DAC;
        }
    } else if (arg = "mppt") {
        dac_mode = DacMode::DIRECT_DAC;
        mppt_enabled = !mppt_enabled;
        mppt_timer = millis();
    } else if (arg = "print") {
        print_output = !print_output;
    } else if (arg = "sweep") {
        dac_mode = DacMode::DIRECT_DAC;
        mppt_enabled = false;
        sweep_dac = true;
        sweep_timer = millis();
    } else {
        Serial.println("Invalid subcommand for toggle");
        Serial.println("Try \"help\"");
    }
}

void select(String &command) {
    String arg = next_arg(command).toLowerCase();

    if (arg == "mode") {
        String selected_mode = next_arg(command).toLowerCase();

        if (selected_mode == "manual") {
            mode = Modes::MANUAL;
        } else if (selected_mode == "auto") {
            mode = Modes::AUTO;
            state = States::STARTUP;
        }
    } else {
        Serial.println("Invalid subcommand for select");
        Serial.println("Try \"help\"");
    }
}

// Interrupt for measuring the RPM
void RPM_Interrupt() {
    rpm_filter_insert(rpm_filter);
}
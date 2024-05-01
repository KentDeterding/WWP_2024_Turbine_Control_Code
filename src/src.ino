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

enum Modes mode = Modes::AUTO;
enum DacMode dac_mode = DacMode::DIRECT_DAC;
enum PitchMode pitch_mode = PitchMode::DIRECT_LA;
enum States state = States::STARTUP;

// Linear Actuator
PA12 myServo(&Serial1, 16, 1);
//PA12 linear_actuator = PA12(&Serial1, 16, 1); // possible replace alias for myServo
const int cut_in_position = 950;
const int feathered_position = 3200;

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dac_value = 50; // 0 - 4095
float targetResistance = 16.0;

// RPM
struct RpmFilter *rpm_filter = new_rpm_filter(6, 8);

struct {
    float prev_power = 0.0;
    int prev_la_pos = 0;
    const int la_step_size = 50;
    uint16_t prev_dac_value = 0;
    float prev_dir = 1;
} mppt_data;

// Timers
unsigned long print_timer;
const unsigned long print_timer_interval = 500;
unsigned long resistance_tracking_timer;
const unsigned long resistance_tracking_interval = 3;
unsigned long mppt_timer;
const unsigned long mppt_interval = 1000;
unsigned long sweep_timer;
const unsigned long sweep_interval = 500;
unsigned long read_timer;
const unsigned long read_interval = 50;
unsigned long state_machine_timer;
const unsigned long state_machine_interval = 100;

// Global State
int dac_step_size = 2;
bool print_output = true;
bool mppt_enabled = false;
float last_power = 0;
bool sweep_dac = false;

struct DigitalFilter *power_filter = new_digital_filter(8);

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
    print_timer = millis() + 3000;
    resistance_tracking_timer = millis();
    read_timer = millis();
}

void loop() {
    if (Serial.available() > 0) {
        String serialInput = Serial.readStringUntil('\n');
        ProcessCommand(serialInput);
    }

    if (print_timer < millis() && print_output) {
        print_timer += print_timer_interval;
        PrintOutput();
    }

    if (read_timer < millis()) {
        read_timer += read_interval;
        digital_filter_insert(power_filter, (float)ina260.readPower());
    }

    if (mode == Modes::AUTO) {
        switch (state) {
            case States::SAFETY:
                myServo.goalPosition(LA_ID_NUM, feathered_position);
                if (safety_switch_closed() && pcc_connected()) {
                    state = States::STARTUP;
                }
                break;
            case States::STARTUP:
                digitalWrite(PCC_RELAY_PIN, HIGH);
                Serial.println("Powering up t-side...");
                delay(3000);

                //while (!myServo.available()) {}
                // TODO: Don't start at optimal pitch in high wind speeds.
                myServo.goalPosition(LA_ID_NUM, cut_in_position);
                //while (myServo.Moving(LA_ID_NUM)) {} // wait for linear actuator to stop moving
                while (abs(myServo.presentPosition(LA_ID_NUM) - myServo.goalPosition(LA_ID_NUM)) > 400) {
                    Serial.println("Delta:\t" + (myServo.presentPosition(LA_ID_NUM) - myServo.goalPosition(LA_ID_NUM)));
                }

                state = States::AWAIT_POWER;

                break;
            case States::AWAIT_POWER:
                // TODO: add a buffer to ensure power is stable
                if (rpm_filter_get(rpm_filter) > 500) {
                    resistance_tracking_timer = millis();
                    myServo.goalPosition(LA_ID_NUM, 700);
                    dac_value = 0;
                    dac.setVoltage(dac_value, false);
                    mppt_timer = millis() + mppt_interval;
                    mppt_data.prev_power = digital_filter_get_avg(power_filter);
                    mppt_data.prev_dir = 1;
                    state = States::POWER_CURVE;
                    digitalWrite(PCC_RELAY_PIN, LOW);
                }
                break;
            case States::POWER_CURVE:
                if (digitalRead(SAFETY_SWITCH_PIN) || !pcc_connected()) {
                    myServo.goalPosition(LA_ID_NUM, feathered_position);
                    state = States::SAFETY;
                    digitalWrite(PCC_RELAY_PIN, HIGH);
                    delay(3000);
                }

                if (ina260.readBusVoltage() < 25)
                    break;

                float load_voltage = ina260.readBusVoltage();
                float load_current = ina260.readCurrent();
                float load_power = ina260.readPower();
                float resistance = load_voltage / ina260.readCurrent();
                float difference = resistance - targetResistance;

                float target_current = load_voltage / targetResistance;

                const float m = 1.59404;

                uint16_t new_dac_value = (int)(target_current / m);

                dac_value = (new_dac_value + dac_value) / 2;

                /*
                if (dac_value > 200) {
                    dac_step_size = 5;
                } else {
                    dac_step_size = 2;
                }

                if (difference > 0) {
                    dac_value += dac_step_size;
                } else {
                    dac_value -= dac_step_size;
                }

                */
                if (dac_value > 4095) {
                    dac_value = 4095;
                } else if (dac_value < 0) {
                    dac_value = 0;
                }
                dac.setVoltage(dac_value, false);

                /*
                if (resistance < 0.1) {
                    targetResistance = 24.0;
                    dac_value = 5;
                    dac.setVoltage(dac_value, false);
                    delay(1000);
                }
                dac.setVoltage(dac_value, false);

                if ((abs(targetResistance - ina260.readBusVoltage() / ina260.readCurrent()) > 0.5))
                    mppt_timer = millis() + mppt_interval;
                if (mppt_timer < millis()) {
                    mppt_timer = millis() + mppt_interval;
                    if (digital_filter_get_avg(power_filter) > mppt_data.prev_power) {
                        mppt_data.prev_power = digital_filter_get_avg(power_filter);
                        targetResistance = targetResistance + (0.3 * mppt_data.prev_dir);
                    } else {
                        mppt_data.prev_power = digital_filter_get_avg(power_filter);
                        mppt_data.prev_dir = mppt_data.prev_dir * -1;
                        targetResistance = targetResistance + (0.3 * mppt_data.prev_dir);
                    }
                    if (targetResistance < 4.0)
                        targetResistance = 4.0;
                }
                */
                    
                break;
        }
    }
}

bool safety_switch_closed() {
    return !digitalRead(SAFETY_SWITCH_PIN);
}

bool pcc_connected() {
    if (digitalRead(PCC_RELAY_PIN) && digitalRead(PCC_STATUS_PIN)) {
        return false;
    }
    return !(
        !digitalRead(PCC_STATUS_PIN) && 
        ina260.readBusVoltage() < 50.0 && 
        rpm_filter_get(rpm_filter) > 200
    );
}

String PadString(String str) {
    while (str.length() < 8) {
        str = " " + str;
    }
    return str;
}

void PrintOutput() {
    String relayState = digitalRead(PCC_RELAY_PIN) ? "High" : "Low";
    String turbineVoltage = digitalRead(30) ? "off" : "on";
    String relayStateStr = PadString(relayState);
    String safetySwitchStr = PadString(digitalRead(SAFETY_SWITCH_PIN) ? "open" : "closed"); // Should shutdown when closed
    String turbineVoltageStr = PadString(turbineVoltage);
    String laTargetStr = PadString(String(myServo.goalPosition(LA_ID_NUM)));
    String laPosStr = PadString(String(myServo.presentPosition(LA_ID_NUM)));
    String la_load_str = PadString(String(myServo.presentLoad(LA_ID_NUM)));
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
            default:
                current_state = "nil";
                break;
        }
    }
    Serial.print("\n\n\n");
    Serial.println("Time:                " + PadString(String(millis())));
    Serial.println("\tFSM:          " + current_state);
    Serial.println("\tRelay State:  " + relayStateStr);
    Serial.println("\tSafety:       " + safetySwitchStr);
    Serial.println("\tT-Status:     " + turbineVoltageStr);
    Serial.println("\tLA Target:    " + laTargetStr);
    Serial.println("\tLA Position:  " + laPosStr);
    //Serial.println("\tLA Load:      " + la_load_str);
    //Serial.println("\tMPPT Enabled: " + mpptStatus);
    Serial.println("\tTarget Res:   " + PadString(String(targetResistance)));
    Serial.println("\tDac:          " + dacValStr);
    Serial.println("\tResistance:   " + resistanceStr);
    Serial.println("\tCurrent:      " + currentStr);
    Serial.println("\tVoltage:      " + voltStr);
    Serial.println("\tPower:        " + powerStr);
    Serial.println("\tRPM:          " + rpmStr);
}

void ProcessCommand(String &serialInput) {
    String command = serialInput;
    String cmd = next_arg(command);

    if (cmd == "help") {
        help();
    } else if (cmd == "set") {
        cmd = next_arg(command);

        if (cmd == "dac") {
            dac_value = next_arg(command).toInt();
            dac.setVoltage(dac_value, false);
            Serial.println("DAC set to " + String(dac_value));
        } else if (cmd == "la") {
            int pos = next_arg(command).toInt();
            myServo.goalPosition(LA_ID_NUM, pos);
            Serial.println("Linear Actuator set to " + String(pos));
        } else if (cmd == "res") {
            targetResistance = next_arg(command).toFloat();
        } else {
            Serial.println("Invalid subcommand for set");
            Serial.println("Try \"help\"");
        }
    } else if (cmd == "toggle") {
        cmd = next_arg(command);

        if (cmd == "pcc") {
            digitalWrite(PCC_RELAY_PIN, !digitalRead(PCC_RELAY_PIN));
        } else {
            Serial.println("Invalid subcommand for toggle");
            Serial.println("Try \"help\"");
        }
    } else if (cmd == "select") {
        cmd = next_arg(command);

    } else if (cmd == "mode") {
        cmd = next_arg(command);

        if (cmd == "manual") {
            mode = Modes::MANUAL;
        } else if (cmd == "auto") {
            mode = Modes::AUTO;
            state = States::STARTUP;
        }
    }
}

// Interrupt for measuring the RPM
void RPM_Interrupt() {
    rpm_filter_insert(rpm_filter);
}
#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>
#include "commands.h"
#include "rpmFilter.h"
#include "digitalFilter.h"
#include "powerFilter.h"

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
const int optimal_pitch = 605;
const int cut_in_position = 1100;
const int feathered_position = 3200;
const int target_rpm = 2900;

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dac_value = 50; // 0 - 4095
float target_resistance = 16.0;

// RPM
struct RpmFilter *rpm_filter = new_rpm_filter(6, 8);

PowerFilter on_filter = PowerFilter(1500, 100);

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
const unsigned long resistance_tracking_interval = 50;
unsigned long mppt_timer;
const unsigned long mppt_interval = 1000;
unsigned long sweep_timer;
const unsigned long sweep_interval = 500;
unsigned long read_timer;
const unsigned long read_interval = 25;
unsigned long regulate_timer;
const unsigned long regulate_interval = 200;
unsigned long time_in_state = 0;

// Global State
int dac_step_size = 2;
bool print_output = false;
bool mppt_enabled = false;
float last_power = 0;
bool sweep_dac = false;
bool delay_set = false;
unsigned int delay_timer;
unsigned int safety_time;
float last_target_res = 32.0;

struct DigitalFilter *power_filter = new_digital_filter(5);
float load_power = 0;

int la_pos;

void setup() {
    Serial.begin(9600);

    // Saftey Switch
    pinMode(SAFETY_SWITCH_PIN, INPUT);

    //Relay
    pinMode(PCC_RELAY_PIN, OUTPUT);
    digitalWrite(PCC_RELAY_PIN, HIGH); // Start with turbine-side powered

    //Linear Actuator
    myServo.begin(32);

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

    if (Serial && print_timer < millis()) {
        print_timer = millis() + print_timer_interval;
        PrintOutput();
    }

    if (read_timer < millis()) {
        read_timer = millis() + read_interval;
        digital_filter_insert(power_filter, (float)ina260.readPower());
    }

    if (on_filter.timer < millis()) {
        on_filter.timer = millis() + on_filter.get_interval();
        on_filter.poll();
    }

    if (mode == Modes::AUTO) {
        switch (state) {
            case States::REGULATE:
                if (digitalRead(SAFETY_SWITCH_PIN) || !pcc_connected()) {
                    myServo.goalPosition(LA_ID_NUM, feathered_position);
                    state = States::SAFETY;
                    digitalWrite(PCC_RELAY_PIN, HIGH);
                    delay(5000);
                } else if (digital_filter_get_avg(power_filter) < 10 && ina260.readBusVoltage() < 100) {
                    state = States::STARTUP;
                } else if (rpm_filter_get(rpm_filter) < target_rpm - 800) {
                  dac_value = 0;
                  target_resistance = 8;
                    state = States::POWER_CURVE;
                }

                if (regulate_timer < millis()) {
                    regulate_timer = millis () + regulate_interval;

                    int cur_rpm = rpm_filter_get(rpm_filter);

                    int difference = cur_rpm - target_rpm;

                    if (300 <= difference) {
                      la_pos += 30;
                    } else if (80 <= difference && difference < 300) {
                        la_pos += 15;
                    } else if (35 <= difference && difference < 80) {
                        la_pos += 3;
                    }

                    if (cur_rpm < target_rpm - 500) {
                        la_pos -= 20;
                    } 
                    else if (cur_rpm < target_rpm - 200) {
                        la_pos -= 15;
                    } else if (cur_rpm < target_rpm - 35) { 
                        la_pos -= 5;
                    }

                    if (la_pos < optimal_pitch)
                        la_pos = optimal_pitch;

                    myServo.goalPosition(LA_ID_NUM, la_pos);
                }

                if (resistance_tracking_timer < millis()) {
                    resistance_tracking_timer = millis() + resistance_tracking_interval;
                    float load_voltage = ina260.readBusVoltage();

                    float target_current = load_voltage / target_resistance;

                    const float m = 1.59404;

                    uint16_t new_dac_value = (int)(target_current / m);
                   
                    dac_value = (new_dac_value + dac_value*4) / 5;

                    if (dac_value > 4095) {
                        dac_value = 4095;
                    } else if (dac_value < 0) {
                        dac_value = 0;
                    }
                    dac.setVoltage(dac_value, false);
                }

                load_power = digital_filter_get_avg(power_filter);
            
                if (load_power < 5000) {
                    target_resistance = 16.0;
                } else if (load_power <  6000) {
                    target_resistance = 11.0;
                } else if (load_power <  9000) {
                    target_resistance =  8.0;
                } else if (load_power < 12500) {
                    target_resistance =  5.5;
                } else if (load_power < 14000) {
                    target_resistance =  5.0;
                } else if (load_power < 24000) {
                    target_resistance =  4.0;
                }

                break;
            case States::SAFETY:
                myServo.goalPosition(LA_ID_NUM, feathered_position);
                if (safety_time + 10000 > millis())
                    break;
                if (safety_switch_closed() && pcc_connected()) {
                    state = States::STARTUP;
                }
                break;
            case States::STARTUP:
                digitalWrite(PCC_RELAY_PIN, HIGH);

                if (!delay_set) {
                    delay_timer = millis() + 6000;
                    delay_set = true;
                    Serial.println("Powering up t-side...");
                }
                if (delay_timer > millis())
                    break;
                delay_set = false;

                myServo.goalPosition(LA_ID_NUM, cut_in_position);
                while (abs(myServo.presentPosition(LA_ID_NUM) - myServo.goalPosition(LA_ID_NUM)) > 400) {}

                dac_value = 0;
                dac.setVoltage(dac_value, false);

                digitalWrite(PCC_RELAY_PIN, LOW);

                state = States::AWAIT_POWER;
                break;
            case States::AWAIT_POWER:
                if (ina260.readBusVoltage() > 5000 && rpm_filter_get(rpm_filter) > 600) {
                    myServo.goalPosition(LA_ID_NUM, optimal_pitch);
                    resistance_tracking_timer = millis();
                    target_resistance = last_target_res;
                    if (target_resistance > 8.0)
                        target_resistance = 16.0;
                    state = States::POWER_CURVE;
                    time_in_state = millis();
                }
                break;
            case States::POWER_CURVE:
                if (digitalRead(SAFETY_SWITCH_PIN) || !pcc_connected()) {
                    myServo.goalPosition(LA_ID_NUM, feathered_position);
                    state = States::SAFETY;
                    digitalWrite(PCC_RELAY_PIN, HIGH);
                    if (target_resistance < 5.6) {
                        last_target_res = target_resistance + 10;
                    } else {
                        last_target_res = 32.0;
                    }
                    safety_time = millis();
                } else if (digital_filter_get_avg(power_filter) < 10 && ina260.readBusVoltage() < 100) {
                    state = States::STARTUP;
                    last_target_res = 32.0;
                } else if (rpm_filter_get(rpm_filter) > target_rpm && ina260.readPower() > 20000) {
                    state = States::REGULATE;
                }

                if (ina260.readBusVoltage() < 25)
                    break;

                if (resistance_tracking_timer < millis() && millis() - time_in_state > 200) {
                    resistance_tracking_timer += resistance_tracking_interval;
                    float load_voltage = ina260.readBusVoltage();

                    float target_current = load_voltage / target_resistance;

                    const float m = 1.59404;

                    uint16_t new_dac_value = (int)(target_current / m);
                    dac_value = (new_dac_value + dac_value) / 2;
                    
                    if (dac_value > 4095) {
                        dac_value = 4095;
                    } else if (dac_value < 0) {
                        dac_value = 0;
                    }
                    dac.setVoltage(dac_value, false);
                }

                load_power = digital_filter_get_avg(power_filter);

                if (millis() - time_in_state < 4000)
                    break;

                if (load_power < 5000) {
                    target_resistance = 16.0;
                } else if (load_power <  6000) {
                    target_resistance = 11.0;
                } else if (load_power <  9000) {
                    target_resistance =  8.0;
                } else if (load_power < 12500) {
                    target_resistance =  5.5;
                } else if (load_power < 14000) {
                    target_resistance =  5.0;
                } else if (load_power < 24000) {
                    target_resistance =  4.0;
                }
                
                myServo.goalPosition(LA_ID_NUM, optimal_pitch);
                    
                break;
        }
    } else {
        if (resistance_tracking_timer < millis()) {
            resistance_tracking_timer += resistance_tracking_interval;
            float load_voltage = ina260.readBusVoltage();

            float target_current = load_voltage / target_resistance;

            const float m = 1.59404;

            uint16_t new_dac_value = (int)(target_current / m);

            dac_value = (new_dac_value + dac_value * 4) / 5;

            if (dac_value > 4095) {
                dac_value = 4095;
            } else if (dac_value < 0) {
                dac_value = 0;
            }
            dac.setVoltage(dac_value, false);
        }
    }
}

void transition_safety() {

}

bool safety_switch_closed() {
    return !digitalRead(SAFETY_SWITCH_PIN);
}

bool pcc_connected() {
    if (digitalRead(PCC_RELAY_PIN) && digitalRead(PCC_STATUS_PIN))
        return false;
    if (ina260.readBusVoltage() < 50 && ina260.readPower() > 100)
        return false;
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
    String turbineVoltage = digitalRead(PCC_STATUS_PIN) ? "off" : "on";
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
    Serial.println("\tSafety:       " + safetySwitchStr);
    Serial.println("\tT-Status:     " + turbineVoltageStr);
    Serial.println("\tLA Target:    " + laTargetStr);
    Serial.println("\tLA Position:  " + laPosStr);
    //Serial.println("\tLA Load:      " + la_load_str);
    //Serial.println("\tMPPT Enabled: " + mpptStatus);
    Serial.println("\tTarget Res:   " + PadString(String(target_resistance)));
    Serial.println("\tDac:          " + dacValStr);
    Serial.println("\tResistance:   " + resistanceStr);
    Serial.println("\tCurrent:      " + currentStr);
    Serial.println("\tVoltage:      " + voltStr);
    Serial.println("\tPower:        " + powerStr);
    Serial.println("\tRPM:          " + rpmStr);
    //Serial.println("\tTarget RPM:   " + target_rpm);
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
            target_resistance = next_arg(command).toFloat();
        } else if (cmd == "rpm") {
            //target_rpm = next_arg(command).toInt();
        } else {
            Serial.println("Invalid subcommand for set");
            Serial.println("Try \"help\"");
        }
    } else if (cmd == "toggle") {
        cmd = next_arg(command);

        if (cmd == "pcc") {
            digitalWrite(PCC_RELAY_PIN, !digitalRead(PCC_RELAY_PIN));
        } else if (cmd == "print") {
            print_output = !print_output;
            if (print_output)
                print_timer = millis();
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
            resistance_tracking_timer = millis();
        } else if (cmd == "auto") {
            mode = Modes::AUTO;
            state = States::STARTUP;
        }
    } else if (cmd == "state") {
        cmd = next_arg(command);

        if (cmd == "startup") {
            state = States::STARTUP;
        } else if (cmd == "power") {
            state = States::POWER_CURVE;
            myServo.goalPosition(LA_ID_NUM, optimal_pitch);
        } else if (cmd == "regulate") {
            state = States::REGULATE;
            regulate_timer = millis();
            la_pos = myServo.presentPosition(la_pos);
        }
    }
}

// Interrupt for measuring the RPM
void RPM_Interrupt() {
    rpm_filter_insert(rpm_filter);
}

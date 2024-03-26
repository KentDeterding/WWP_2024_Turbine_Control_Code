# 1 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
# 2 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 3 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 4 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 5 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 6 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 7 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2






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

struct Filter* rpm_filter = CreateFilter(10, 14);

// Global State
bool trackResistance = false;
int dacStepSize = 100;


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
    myServo.movingSpeed(0 /* ID number of the linear actuator*/, 750);

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
    pinMode(33 /* Relay control pin*/, 1);

    // Start RPM Tracking
    attachInterrupt(((29) < 55 ? (29) : -1), RPM_Interrupt, 3);

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

    if (printTimer < millis()) {
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
    Serial.println("LA Position: \t" + String(myServo.presentPosition(0 /* ID number of the linear actuator*/)));
    String relayState = digitalRead(33 /* Relay control pin*/) ? "High" : "Low";
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
    String arg = NextArg(&command);

    if (arg.toLowerCase() == "dac") {
        dacValue = NextArg(&command).toInt();
        dac.setVoltage(dacValue, false);
        Serial.println("DAC set to " + String(dacValue));
    } else if (arg.toLowerCase() == "la") {
        int pos = NextArg(&command).toInt();
        myServo.goalPosition(0 /* ID number of the linear actuator*/, pos);
        Serial.println("Linear Actuator set to " + String(pos));
    } else if (arg.toLowerCase() == "res") {
        targetResistance = NextArg(&command).toFloat();
    }
}

//
void Switch(String command) {
    String arg = NextArg(&command);

    if (arg.toLowerCase() == "pcc") {
        digitalWrite(33 /* Relay control pin*/, !digitalRead(33 /* Relay control pin*/));
        Serial.println("Relay set to " + digitalRead(33 /* Relay control pin*/) ? "High" : "Low");
    }
}

// Interrupt for measuring the RPM
void RPM_Interrupt () {
    int time = (int)micros();
    Insert(rpm_filter, time);
}

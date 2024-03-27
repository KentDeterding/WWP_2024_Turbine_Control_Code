# 1 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
# 2 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 3 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 4 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 5 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 6 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2
# 7 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino" 2






// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
uint16_t dacValue = 50; // 0 - 4095
float targetResistance = 8;

// RPM

struct Filter* rpm_filter = CreateFilter(10, 8);

// Timers
unsigned long printTimer;
unsigned long printTimerInterval = 1000;
unsigned long resistanceTracingTimer;
unsigned long resistanceTrackingInterval = 100;

// Global State
bool trackResistance = false;
int dacStepSize = 100;
bool printOutput = false;


void setup () {
    Serial.begin(9600);
    while (!Serial) { // Wait so serial monitor can be opened
        delay(10);
    }
    Serial.println("Starting up...");
    bool success = true;

    /*

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



    */
# 93 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
    Serial.println("Type \"help\" for a list of commands");

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

    /*

    if (digitalRead(Safety_Switch_Pin) == HIGH) {

        myServo.goalPosition(LA_ID_NUM, 0);

        dacValue = 4095;

        dac.setVoltage(dacValue, false);

    }

    */
# 119 "C:\\Users\\Kent4\\Projects\\Wildcat_Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\src.ino"
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
    Serial.print("\n\n\n");
    Serial.println("Time:          \t" + PadString(String(millis())));
    String relayState = digitalRead(33 /* Relay control pin*/) ? "High" : "Low";
    Serial.println("\tRelay State: " + PadString(relayState));
    String turbineVoltage = digitalRead(30) ? "High" : "Low";
    Serial.println("\tT-Side Volt: " + PadString(turbineVoltage));
    Serial.println("\tLA Position: " + PadString(String(myServo.presentPosition(0 /* ID number of the linear actuator*/))));
    Serial.println("\tDac:         " + PadString(String(dacValue)));
    Serial.println("\tCurrent:     " + PadString(String(ina260.readCurrent())));
    Serial.println("\tVoltage:     " + PadString(String(ina260.readBusVoltage())));
    Serial.println("\tPower:       " + PadString(String(ina260.readPower())));
    Serial.println("\tRPM:         " + PadString(String(GetRpmBuffered(rpm_filter))));
}

void ProcessCommand (String &serialInput) {
    String test = "set dac 100";
    Serial.println(NextArg(test));
    Serial.println(NextArg(test));
    Serial.println(NextArg(test));

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
        myServo.goalPosition(0 /* ID number of the linear actuator*/, pos);
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
    Serial.println("Toggling " + arg);

    if (arg == "pcc") {
        digitalWrite(33 /* Relay control pin*/, !digitalRead(33 /* Relay control pin*/));
        Serial.println("Relay set to " + digitalRead(33 /* Relay control pin*/) ? "High" : "Low");
    } else if (arg = "res") {
        trackResistance = !trackResistance;
    } else if (arg = "print") {
        printOutput = !printOutput;
    } else {
        Serial.println("Invalid subcommand for switch");
        Serial.println("Try \"help\"");
    }

    arg = NextArg(command).toLowerCase();
    Serial.println("Toggling " + arg);
}

// Interrupt for measuring the RPM
void RPM_Interrupt () {
    int time = (int)micros();
    Insert(rpm_filter, time);
}

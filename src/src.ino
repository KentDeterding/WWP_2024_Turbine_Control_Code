#include <Wire.h>
#include <PA12.h>
#include <Adafruit_INA260.h>
#include <Adafruit_MCP4725.h>


#define LA_ID_NUM 0 // ID number of the linear actuator


// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;
int dacValue = 0;

// Timers
unsigned long printTimer = 0;


void setup () {
    Serial.begin(9600);
    Serial.println("Starting up...");
    bool success = true;

    //Linear Actuator
    myServo.begin(32);
    delay(10);
    myServo.movingSpeed(LA_ID_NUM, 750);
    int pos = myServo.presentPosition(LA_ID_NUM);
    if (pos >= 0 && pos <= 4095) {
        Serial.println("Linear actuator ready");
    } else {
        Serial.println("Linear actuator error");
        success = false;
    }

    //INA260
    ina260.begin();
    delay(10);
    if (ina260.readPower()) {
        Serial.println("INA260 ready");
    } else {
        Serial.println("INA260 error");
        success = false;
    }
    
    //DAC
    dac.begin(0x60);
    delay(10);
    dac.setVoltage(dacValue, false);
    Serial.println("DAC ready");

    if (success) {
        Serial.println("Setup complete");
    } else {
        Serial.println("Setup failed");
    }
}

void loop () {
    if (printTimer < millis()) {
        printTimer += 100;
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
    Serial.println("Time: \t" + PadString(String(millis())));
    Serial.println("Dac: \t" + PadString(String(dacValue)));
    Serial.println("Power: \t" + PadString(String(ina260.readPower())));
    Serial.println("LA Position: \t" + PadString(String(myServo.presentPosition(LA_ID_NUM))));
}
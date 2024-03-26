#include <Arduino.h>
#include "types.h"

// Get the next argument from a string and remove it
String NextArg(String* command) {
    String arg = (*command).substring(0, (*command).indexOf(" "));
    *command = (*command).substring((*command).indexOf(" ") + 1);
    return arg;
}

Command MatchCommand(String command) {
    if (command == "set") {
        return Command::SET;
    } else if (command.toLowerCase() == "switch") {
        return Command::SWITCH;
    } else if (command.toLowerCase() == "help") {
        return Command::HELP;
    } else {
        return Command::INVALID;
    }
}

// Help menu
void Help() {
    /*
    Serial.println("List of valid commands:");
    Serial.println("");
    Serial.println("set <target> <value>");
    Serial.println("    | dac <int>");
    Serial.println("    | res <float>");
    Serial.println("    | la <int>");
    Serial.println("    | pitch <float>");
    Serial.println("switch <traget>");
    Serial.println("    | pcc");
    Serial.println("    | res");
    */
    // TODO: Fill in commands
}
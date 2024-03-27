#include <Arduino.h>
#include "types.h"

// Get the next argument from a string and remove it
String NextArg(String &command) {
    String arg = command.substring(0, command.indexOf(" "));
    command = command.substring(command.indexOf(" ") + 1);
    return arg;
}

Command MatchCommand(String command) {
    if (command == "set") {
        return Command::SET;
    } else if (command.toLowerCase() == "toggle") {
        return Command::TOGGLE;
    } else if (command.toLowerCase() == "help") {
        return Command::HELP;
    } else {
        return Command::INVALID;
    }
}

// Help menu
String Help() {
    // TODO: Fill in commands
    String string =   "List of valid commands:\n";
            string += "set <target> <value>\n";
            string += "    | dac <int>\n" +
            string += "    | res <float>\n" +
            string += "    | la <int>\n" +
            string += "    | pitch <float> (not implemented)\n" +
            string += "toggle <traget>\n" +
            string += "    | pcc\n" +
            string += "    | res (resistance tracking)\n" +
            string += "    | print\n";
    return string;
}
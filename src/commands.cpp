#include <Arduino.h>
#include "types.h"

// Get the next argument from a string and remove it
String next_arg(String &command) {
    String arg = command.substring(0, command.indexOf(" "));
    command = command.substring(command.indexOf(" ") + 1);
    return arg;
}

// Help menu
String help() {
    // TODO: Fill in commands
    String string =   "List of valid commands:\n";
            string += "set <target> <value>\n";
            string += "    | dac <int>\n" +
            string += "    | res <float>\n" +
            string += "    | la <int>\n" +
            string += "toggle <target>\n" +
            string += "    | pcc\n" +
            string += "mode <new mode>\n" +
            string += "    : auto\n" +
            string += "    : manual\n";
    return string;
}
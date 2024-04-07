#ifndef commands_h
#define commands_h 

#include <stdlib.h>
#include <Arduino.h>
#include "types.h"

String next_arg(String &command);

Command match_command(String command);

String help();

#endif
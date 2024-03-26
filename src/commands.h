#ifndef commands_h
#define commands_h 

#include <stdlib.h>
#include <Arduino.h>
#include "types.h"

String NextArg(String* command);

Command MatchCommand(String command);

String Help();

#endif
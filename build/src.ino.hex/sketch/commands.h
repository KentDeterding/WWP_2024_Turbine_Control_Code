#line 1 "C:\\Users\\Kent4\\Projects\\Wind_Power\\WWP_2024_Turbine_Control_Code\\src\\commands.h"
#ifndef commands_h
#define commands_h 

#include <stdlib.h>
#include <Arduino.h>
#include "types.h"

String NextArg(String &command);

Command MatchCommand(String command);

String Help();

#endif
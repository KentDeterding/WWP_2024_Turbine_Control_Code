#ifndef powerFilter_h
#define powerFilter_h 
#include <Arduino.h>

using namespace std;

class PowerFilter {
    private:
        const int min = 0;
        int max;
        int count;
        unsigned long interval;
    
    public:
        unsigned long timer;

        PowerFilter(int total_time /*ms*/, int polling_interval /*ms*/) {
            max = total_time / polling_interval;
            count = 0;
            interval = polling_interval;
        }

        void reset() {
            count = 0;
        }

        void poll() {
            if (!digitalRead(30)) {
                count++;
            } else {
                count--;
            }

            if (count < 0) {
                count = 0;
            } else if (count > max) {
                count = max;
            }
        }

        bool is_on() {
            if (count == max) {
                return true;
            }
            return false;
        }

        unsigned long get_interval() {
            return interval;
        }
};

#endif
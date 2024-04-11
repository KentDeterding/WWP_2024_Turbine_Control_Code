enum Modes {
    MANUAL,
    AUTO,
    TEST
};

enum DacMode {
    DIRECT_DAC,
    RESISTANCE
};

enum PitchMode {
    DIRECT_LA,
    THETA
};

struct ManualState {
    enum DacMode dac_mode;
    float target_resistance;
    enum PitchMode pitch_mode;
    float goal_theta;
};
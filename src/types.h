enum Command {
    INVALID,
    HELP,
    SET,
    TOGGLE,
    SELECT
};

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

enum States {
    STARTUP,
    AWAIT_POWER,
    SAFETY,
    POWER_CURVE,
    REGULATE
};
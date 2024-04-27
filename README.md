# WWP_2024_Turbine_Control_Code
 
# to-do
- T - MPPT for Load
- Safety Switch
- PCC Disconnect
- Better output in the serial monitor
//- Diff Pressure Sensor Model#: SEN0343
- Make DAC ergonimics better (struct?)

# Inputs

### RPM Square Wave
Pin Num = 29

### PCC Disconnect
Pin Num = 30
    30 High = TS High Voltage

### Safety Switch
Pin Num = 11
An open should turn the turbine off.


# Outputs

### Relay
Pin Num = 33
High = Relay Closed = Turbine-Side is powered by Load

### Fan
Pin Num = 20

## I2C

### Ina 260
i2c address = 0x40

### DAC (MCP4725)
i2c address = 0x64

### Turbine
Open for future connections (pitot tube, accelerometer)


## Linear Actuators
TX = TX1  = D1
RX = RX1  = D0
En = SCL1 = D16


# Loop Psudo-Code
- Check input
- Execute Command
    - manual mode
        - set dac
        - set la raw
        - set la pitch
        - change relay position
    - test mode
        - sweep dac
        - sweep pitch
        - sweep dac & pitch
        - sweep dac, pitch, & windspeed
- Print to serial port
- One of
    - manual mode
        - track load res
    - state machine
# WWP_2024_Turbine_Control_Code
 
# to-do
- Test relay control
- Read RPM
- MPPT for Load
- Data recording
- Better output in the serial monitor
- Diff Pressure Sensor Model#: SEN0343
- PCC Disconnect    Pin 30 high = turbine side high voltage
- Safety Switch

# Inputs

### RPM Square Wave
Pin Num = D29

### PCC Disconnect
Pin Num = D30

### Safety Switch
Pin Num = D11


# Outputs

### Relay
Pin Num = D33

### Fan
Pin Num = D20

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
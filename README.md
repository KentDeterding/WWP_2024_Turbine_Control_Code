# WWP_2024_Turbine_Control_Code
 
# to-do
- Add power-on time filter for turbine side in await power
- Improve Startup reliability
- Dynamic Step size for dac
- Regulation
- Better output in the serial monitor

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

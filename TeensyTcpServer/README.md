# Tcp Server (Arduino File)

The interaction between the TCP Server (.ino) and the TCP Client (.py) are held by bidirectional messages following the protocol below:

Requests from server consist in a single integer which binary mask encodes information about the device ID, type of instruction and an aditional 15 bits of data (to specify the set velocity to a particular driver or group of drivers)



Here's the description of each device's ID and instructions' ID

| Device ID | Device Description | Instruction ID |      Instruction Description      |
|:---------:|:------------------:|:--------------:|:---------------------------------:|
| 10        | 10-DOF             | 10             | Gyroscope Information (x,y,z)     |
|           |                    | 11             | Accelerometer Information (x,y,z) |
|           |                    | 12             | Magnetometer Information (x,y,z)  |
|           |                    | 13             | Pressure and Temperature          |
|           |                    |                |                                   |
|           |                    |                |                                   |
|           |                    |                |                                   |
|           |                    |                |                                   |
|           |                    |                |                                   |

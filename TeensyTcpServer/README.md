# Tcp Client (Arduino File)

The interaction between the TCP Server (.ino) and the TCP Client (.py) are held by bidirectional messages following the protocol below:

Petition format from the client to the server is a single 4 digit number, where the two first digits from left to write describe the device ID and the last two digits describe the instruction ID 

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

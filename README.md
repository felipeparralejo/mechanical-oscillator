# How does the system work?
This is a __Mechanical Oscillator built in C with 3D printed parts__.

![Mounted System Image](./Mounted%20System.jpg)

The motor is commanded using a PWM-based driver fed with signals obtained using timers (`TCC0` and `TCC1` in this case). This allows to generate oscillations of controllable frequency and peak amplitude that can be changed in real time (because the registers are being directly accessed, without third-party libraries).

The system is based on a Nema17 motor and a custom-designed linear actuator that can be 3D printed: [Nema17 Linear Actuator with Couplings](https://www.thingiverse.com/thing:6530349).

## Limitations
These limitations are based on the maximum and minimum PWM signals frequencies from timers, and the length of the 3D printed rack.

|      | Frequency (Hz) | Peak Amplitude (mm) |
|:----:|:--------------:|:-------------------:|
| Min. |    0.001397    |       0.007314      |
| Max. |       24       |         ~20         |

## Hardware
* The current version of the software provided works with SAMD21 microcontrollers with network capabilities. The `Arduino MKR1000` is used in this case.
* `Nema17 17HS3401` stepper motor with 1.8◦ stepping angle and 1.3 A rated current
* `DRV8825` stepper motor driver with microstepping capabilities
* `100 μF` capacitor
* `12V, 2A` power supply
* Multimeter

A basic tutorial on how to build the electronic circuit and calibrate the `VREF` pin of the motor driver can be found in [DRV8825 Tutorial](https://lastminuteengineers.com/drv8825-stepper-motor-driver-arduino-tutorial/). For the tests carried out, the value used for the DRV8825 was `VREF=0.47V`.

## Software
The C code provided in the `Stepper_MKR1000.ino` file contains the loop function and the header file `Stepper_WiFi.h` contains the functions needed to create the WiFi access point (or to connect to an existing one), to handle user connections (only one client allowed at a time) and to send messages and receive orders from client.

A Python script `client.py` is provided which automatically connects to the server using a socket telnet connection, and that allows to set new amplitude and frequency values on the go (waiting for server OK responses), and also allows to stop the oscillation at the extremes (`-A` and `+A`).

## Server loop
1. When the Arduino is powered on, it creates an access point called `MKR1000` and waits for clients to connect.
2. When a client has established a connection to the server (connecting to the access point and launching `client.py`) the loop function of the microcontroller is started. The motor starts moving at `f=1.0Hz` and peak amplitude value set by the client when connecting.
3. The server waits for the client to send a new frequency (in `Hz`), peak amplitude (in `mm`) or to command it to stop at the extremes of the oscillation.
4. When the client demands a change of the parameters of the oscillation, the server waits to be in the equilibrium point of the oscillation, changes the value, and sends an `OK` message to the client.

## Client loop
1. When the client connects to the server, it asks for the peak amplitude desired (in `mm`) and the oscillator starts moving with `f=1.0 Hz` and the amplitude set by the client.
2. The client waits for a new frequency value to be sent to the server (in `Hz`), a new peak amplitue value (sending `A`) or commanding the oscillator to stop at the extremes (sending `X`).
3. Everytime the frequency or amplitude is changed, the values are stored in a `pandas.DataFrame` with a timestamp for synchronization purposes. This is saved in the file `freqs_amplitudes.csv` when the server is stopped (Ctrl+C) or an error occurs.

# How to change to A4988 Motor Driver?
The motor can also be controlled using the `A4988` driver. A basic tutorial on this driver can be found in [A4988 Tutorial](https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/). The `MICROSTEPPING` Arduino variable in `Stepper_MKR1000.ino` must be changed to `16`, the maximum microsteps provided by the driver.

# How to connect to an existing network?
By default, the Arduino creates an access point called `MKR1000` but it can also be configured to connect to an existing network providing the SSID and password of the network (see `Stepper_WiFi.h` for more information).

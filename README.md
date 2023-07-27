
# About
🚨Work in progress🚨

It's my spin on a tracking device software. Its main purpose was to use parts I had laying around and try build working tracker. That's why hardware used may not make sense. 

# Goal

# Software
There are 2 main python scripts that are essential for my current setup.
* tracker.py -- brain of the operation, tracking program
* buttons.pyw -- script responsible for handling PiTFT's buttons run as a service in the background

I'm not a programmer. All the code you'll see I treat as a learning experience. 
Messy and suboptimal code ahead!

# Hardware 
Most important parts:
* Raspberry Pi Zero W
* Adafruit PiTFT 2.2"
* Seeedstudio LoRa-E5 mini
* TMC2209 stepper motor driver
* Stepper motor -- pan
* Servo -- tilt

Later I'll add Cad files of the whole assembly with case

# What's working?
- Azimuth and inclination calculation -- with my limited testing looks like it's working. **Needs more testing**
- Turning stepper motor
- Saving tracker's position to a file for later use
- TCP mavlink data stream via Raspberry Pi's access point


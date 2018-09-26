# Wireless MIDI Gloves

A digital instrument that is controlled with hand movements and can be played on any surface! This project was created for a senior project design class.

## A Summary of How it Works

Our digital instrument literally fits like a glove! The idea behind the project was create an instrument that "desk-tappers" and "air-drummers" would have fun using, as well as experienced muscians. We incorporated custom-made velostat force sensors on the finger tips of the glove so that the user can tap their fingers and play a sound. A 6-axis IMU is also placed on the glove so that gestures such as "twisting" and "swiping" can control the glove. BLE MIDI technology allows the user to play free of cords without sacraficing latency. The Fluidsynth API also allows for countless instruments to be played and customization of the sounds.

## Getting Started

We will only cover the software aspects of this project and also the functionality of how the glove works so we will not be discussing the hardware or design of the glove in detail.

Also, this page will only cover the software on the gloves and does not cover the software on the Raspberry Pi, which is loaded with Fluidsynth, to interpret and play the MIDI notes.

### Glove Components

This is a general list of components we used to build the glove. The list does not include resistors or other electrical components that may be needed.

```
    - 2 Adafruit Feather Microcontrollers w/ BLE
    - 1 Raspberry Pi 3
    - 1 Sheet of Velostat
    - 2 6-Axis IMUs
    - 2 SPST switches
    - 1 LCD Screen
    - 1 Rotary Encoder
```
### Installation

The Arduino IDE is needed to upload the software to the gloves.

The folders correspond to the different software loaded into each glove:
```
DrumGlove_RightHand_MPU6050   -----> Upload into the right-hand glove
DrumGlove_LeftHand_LIS3DH -----> Upload into the left-hand glove
```
Reason for two different files:
Our team used the "6-Axis MPU6050 IMU" for the right-hand glove and the "3-Axis LIS3DH IMU" for the left-hand glove. Since we used two different chips for each hand we needed seperate software for each hand. We had problems with the second MPU6050 IMU and switched it out last minute and needed seperate code to enable the LIS3DH IMU. The left hand does not have the "twisting" function but still has the "swiping" function.

If you have two MPU6050s on hand then you can use the code for the right-hand glove on both hands.

## How to Use

The glove works by accepting "combinational inputs", meaning the user will have to tap multiple fingers (2 or more) at once to play a sound. This was done to avoid accidental taps and also to give the user more note posibilities (4 notes vs 4! notes). For example, tapping the index+middle finger on the right hand will play a "snare drum" sound. As the code is right now, there are only 4 combinations with the palm sensor being a single input note (palm sensor only plays one note and needs no combinations).

The IMU will allow for the chaning of the instrument and for pitch-bend capabitlies. On the right-hand there are two velostat flex sensors on the index- and middle-finger. Flexing the middle finger activates the "channel change" function and by swiping left or right the user can change the instrument. Flexing the index finger activates the "pitch-bend" function and by twisting the wrist left or right will bend the current note being played (Bends notes played on left hand as well).

The SPST switch will turn on/off the force sensors. By turning off the sensors, no input will be accepted and the user can freely use their hands without playing sounds.

## Authors

* **Wesley Jonson** - *Glove Sotware and Glove Design* - [WJonson](https://github.com/WJonson)
* **Colton DeMarr** - *Raspberry Pi Software*
* **Tengrithy Khling** - *Glove Design*

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* **Felipe Tonello** - Created BLE MIDI functionality for Bluez
	https://github.com/ftonello
* **Jeff Rowberg** - Created software for MPU6050 IMU
	https://github.com/jrowberg
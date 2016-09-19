# razor-madgwick-ahrs

This is a quaternion-based firmware for SparkFun 9-DOF Razor IMU (SEN-10736). It can be used to get orientation in quaternion form at a 100 Hz rate.

The firmware is a modification of the original work by [Peter Bartz](https://github.com/ptrbrtz/razor-9dof-ahrs). Instead of the DCM algorithm, it uses a modified version of [Madgwick's algorithm](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/). The modifications make the algorithm a bit faster, so it can run at 100 Hz on the ATMEGA328p microcontroller.

The results have been verified by attaching the IMU to a rigid trihedron with optical markers, and capturing the motion by means of a 12-camera OptiTrack system. With the sensors properly calibrated, the obtained accuracy is about one degree.

## Calibration
A sample calibration file using data from the datasheets (**Calibration.h**) is provided in the **Calibrate** folder.

This calibration file does not provide accurate results. In order to perform a proper calibration, the motion capture system must be used, although simpler methods such as that described by Peter Bartz can provide a fair approximation. The code necessary to perform the calibration is planned to be uploaded in the future.

The calibration is applied by means of the **Calibrate.ino** sketch. If the **WRITE** flag is set to *true*, the ID number and calibration data are written to the onboard EEPROM, otherwise the script just outputs the ID number and the factory OSCCAL calibration value (useful when the 8 MHz oscillator needs recalibration).

## Usage
The main file in the **Firmware** folder contains a description of the accepted commands.

A sample program (**RazorAHRS.m**) is included in the **MATLAB** folder. Prior to running the example, edit the serial port name in the **RazorAHRS.m** file. In order to get proper visualization, the axes printed on the PCB should point as follows:

1. X axis to the left, parallel to the monitor.
2. Y axis towards the monitor screen.
3. Z axis pointing upwards.

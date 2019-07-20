# LSM6DSO

![LSM6DSObreadboard](https://user-images.githubusercontent.com/6698410/61259719-45a1a100-a730-11e9-8fb8-febeca2fea61.jpg)
*This is the LSM6DSO accel/gyro on a breakout board made for the LSM6DSM as master to the LIS2MDL magnetometer and LPS22HB barometer*

The LSM6DSO is the successor to the LSM6DSM combination accel/gyro, currently one of the most accurate and stable "cell-phone" sensors available. When coupled with an LIS2MDL magnetometer, either of these accel/gyros combinations can [routinely](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf) produce absolute orientation estimation with sub one-degree rms heading accuracy with proper calibration. The LSM6DSO also embeds a finite state machine which enables machine learning so that user-defined motion and gestures can be processed and recognized quickly and with ultra-low power usage. From the data sheet:

*"The LSM6DSO can be configured to generate interrupt signals activated by user-defined motion patterns. To do
this, up to 16 embedded finite state machines can be programmed independently for motion detection such as
glance gestures, absolute wrist tilt, shake and double-shake detection."*

**The basic sketch:** configures the LSM6DSO as a master to a LIS2MDL slave magnetometer and LPS22HB slave barometer. The sketch configures all sensors with full-scale range, sample rate, low-pass filtering, etc. The LSM6DSO manages the sensors in master mode, reading their data on a dedicated master I2C bus so that interrupt-driven fast burst reading of LSM6DSO registers via I2C by the host is all that is required to access all of the sensor data. This could be streamlined further by storing all data in the 9 KByte FIFO and reading hundreds of data sets with timestamps at once. This efficient data pipeline facilitates subsequent data processing on the LSM6DSO and detection of specific and complicated motions via the finite state machine function.

The basic sketch uses a simple method to calibrate the sensors, scales the sensor data, uses Madgwick open-source sensor fusion to combine the data into an absolute orientation estimation with yaw (heading), pitch and roll output to the serial monitor along with the scaled sensor data and environmental data such as pressure (altitude) and temperature.

**The EmbeddedFunctions** sketch implements some of the motion detection functions embedded in the LSM6DSO including sleep/wake, single tap detection, tilt detection, and 6D orientation detection. There are others like step detection and step counting, double tap, significant motion detection, and free-fall detection that I haven't implemented yet but it is straightforward to do so. The sketch should serve as an illustration of the method. There is a lot of capability for gesture detection and significant motion detection already available here. The finite state machine allows custom gesture- and motion-detection, and this will be explored next.

I did run into a problem with implementing system sleep. I used the same method (even though the LSM6DSM and LSM6DSO are quite different) that allows all of these sensors and MCU to sleep with a total current usage of ~11 uA with the LSM6DSM, but here there is some activity that pulses to 2500 uA every 3 seconds that I do not understand yet. This happens whether I use passthrough to set all slave sensors into idle mode (the LSM6DSM method) or use the sleep/wake activity function embedded in the LSM6DSO. One possibility is that INT2 in this design is connected to the magnetometer interrupt (which is not activated). This is not a problem with the LSM6DSM, but maybe it is with the LSM6DSO. I redesigned the breakout to disconnect INT2 from the magnetometer interrupt and expose it to the user with a 100 KOhm pull down to GND. This should satisfy the admonition to ground INT2 when performing certain operations like passthrough, as well as allow the activity interrupts to be routed to one of two interrupts for easier management.

Subsequent sketches reposited here will explore the use of the finite state machine for machine learning and low-power gesture recognition, etc.

All sketches use the Dragonfly development board as MCU, which is capable of ultra-low power operation with just 2.1 uA current usage in fast-wakeup STOP mode. The basic sketch shows how to put the sensors to sleep and wake them up such that the total current usage for the system above in lowest-power sleep mode is ~11 uA (well, at least for the LSM6DSM for now).



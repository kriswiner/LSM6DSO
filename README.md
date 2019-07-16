# LSM6DSO

![LSM6DSObreadboard](https://user-images.githubusercontent.com/6698410/61259719-45a1a100-a730-11e9-8fb8-febeca2fea61.jpg)

The LSM6DSO is the successor to the LSM6DSM combination accel/gyro, currently one of the most accurate and stable "cell-phone" sensors available. When coupled with an LIS2MDL magnetometer, either of these accel/gyros combinations can [routinely](https://hackaday.com/wp-content/uploads/2019/03/hackaday_journal-gregorytomasch_kriswiner-heading_accuracy_using_mems_sensors.pdf) produce absolute orientation estimation with sub one-degree rms heading accuray with proper calibration. The LSM6DSO also embeds a finite state machine which enables machine learning so that user-defined motion and gestures can be processed and recognized quickly and with ultra-low power usage. From the data sheet:

*"The LSM6DSO can be configured to generate interrupt signals activated by user-defined motion patterns. To do
this, up to 16 embedded finite state machines can be programmed independently for motion detection such as
glance gestures, absolute wrist tilt, shake and double-shake detection."*

**The basic sketch:** configures the LSM6DSO as a master to a LIS2MDL slave magnetometer and LPS22HB slave barometer. The sketch configures all sensors with full-scale range, sample rate, low-pass filtering, etc. The LSM6DSO manages the sensors in master mode, reading their data on a separate master I2C data bus so that simple batch register reading of the LSM6DSO is all that is required to access all of the sensor data. This data pipeline facilitates subsequent data processing on the LSM6DSO and detection of specific and complicated motions via the finite state machine function.

The basic sketch uses a simple method to calibrate the sensors, scales the sensor data, uses Madgwick open-source sensor fusion to combine the data into an absolute orientation estimation with yaw (heading), pitch and roll output to the serial monitor along with the scaled sensor data and environmental data such as pressure (altitude) and temperature.

Subsequent sketches reposited here will explore the use of the finite state machine for machine learning and low-power gesture recognition, etc.

All sketches uses the Dragonfly development board as MCU, which is capable of ultra-low power operation with just 2.1 uA current usage in fast-wakeup STOP mode. The basic sketch shows how to put the sensors to sleep and wake them up such that the total current usage for the system in lowest-power sleep mode can be less than 8 uA.



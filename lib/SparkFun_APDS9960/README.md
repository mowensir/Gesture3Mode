# SparkFun APDS9960 vendored library

This directory contains SparkFun APDS-9960 Arduino Library version 1.4.1,
copied from the provided SparkFun reference project. The upstream source
attribution and `LICENSE.md` are retained.

Local changes for this project:

- `init()` uses the application's initialized global `Wire` bus instead of
  calling parameterless `Wire.begin()`, preserving the shared/custom bus setup.
- I2C reads verify transmission and received byte counts; block writes use
  `Wire.write()` for the payload.
- `readGestureNonBlocking()` polls and drains FIFO data without waiting in a
  loop. It reuses SparkFun's gesture processing/decoding methods and discards
  captures that overflow the 32-sample buffer.
- `readDeviceID()` is exposed so the application can report and validate the
  supported chip IDs before initialization writes.

The original blocking `readGesture()` API remains available, but this project
uses `readGestureNonBlocking()`.

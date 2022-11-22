# Tracker: film remote vehicles using location telemetry

The purpose of this program is to film UAVs and other remote vehicles that transmit telemetry.
When testing UAVs, it is desirable to have footage of their flights, especially in cases of divergent behavior.
This program automates this task.

The basic setup is:

  * camera on a pan/tilt turret
  * IMU in the base that keeps track of camera base location and orientation
  * telemetry receiver on the ground
  * UAV that transmits its location in telemetry

Initial implementation uses serial-port VISCA cameras and ArduPilot for local and telemetry IMUs.
I expect to add additional backends as this program gains users.
# Instructions #

- Always start the robot in position mode.
- After startup, you can switch the controller to enter another control mode.
- Take sure that the dynamic model is accurate, so that the robot does not drift away after you switch from position mode to one of the impedance control strategies (20 or 30).
- Do not publish any control commands, after the switch is done properly.

Not complying to this rules could cause 'Interpolation Error', 'CmdCrtPos not properly initialized' or the like on the KRC side.

# ATTENTION: #
The cycle time is set to 5ms on the PC side to exclude timing issues.
Change the cycle time in the KRL script accordingly:
friOpen(5);


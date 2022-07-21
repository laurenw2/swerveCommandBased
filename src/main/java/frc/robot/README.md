Includes:

8 SparkMaxes (can be changed)
4 absolute encoders (analog inputs on RoboRio) [makes sure modules properly oriented @ start of match, measures module rotation] (MA3, MAG encoders)
8 interior encoders (within motors)
1 ADXRS450 gyro
A PID controller (just Proportional) which controls each turning motor
Lots of constants that need very very immediate changing


Logic Workflow:

1. Joystick inputs: given three numbers representing the desired xSpd, ySpd, and turning speed of the robot (field relative)
2. Convert values to robot reference frame (Gyro to determine robot heading)
3. Use geometry to calculate the exact speed and angle for each  wheel to attain the new values (the Chassis speeds)
4. Apply wheel speed and angle to modules (SwerveModuleStates)
5. hooray

Needed Before it Works:
1. change constants
2. tune PID
3. find parts necessary (could prove arduous)
4. make necessary measurements (wheel diameters [inches], gear ratios, encoder offset [radians], et multae ceterae)
5. set maximum values (safety!)
6. look at deadband values if you really want, but it's pretty good where it's at
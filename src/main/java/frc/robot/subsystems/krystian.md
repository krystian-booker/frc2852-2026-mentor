For our FRC robot I'd like to have top tier tuning for our PID and Feed forward systems on our robot. In an attempt to accomplish this goal this year I attempted to write SysId scripts which could record data to a csv, then be imported into MatLab where we would perform automatic calibration and be able to use those values on our robot. 
My attempts at this weren't great, you'll see them in the tuning folder and in the subsystems and I think could be mostly ignored other than viewing the general idea of what I was going for.
I'd like you to put together a new library or couple of java files I can reuse year after year to perform this same task. The great part about this task is there really aren't that many types of mechanisms we use every year in FRC, we use:
- Swerve drive
- Position based Elevators
- Position based Turrets
- Position based Pivoting arms
- Velocity based rotating mechanisms

My goal for this would be that I write a subsystem (For this implementation we only need to support the TalonFX motor class, our team only uses the CTRE Kraken X60 and CTRE Kraken X44 motors). 
Then we have some way to add or call this library in our subsystem and whatever setup would be needed, like telling it what kind of mechanism it is, the min and max travel distance (for position based), maybe a max velocity, and whatever else would be needed to perform the SysId commands for this subsystem. 
Maybe the easiest path for this would be to have a new Subsystem class that extends the existing subsystem so we can even have the sysId commmands predefined, but we'd still need a way to define for our new code what this mechanism is and its parameters.
This will need to work for swerve drive as well, we always use the CTRE Swerve Drive Template code, which can be found in the CommandSwerveDrivetrain.java, I'm not sure if we would want to perform a SysId for each azimuth motor and drive motor or if it would be better to have the same tuning on each set of motors, I'll leave that decision up to you.

We then run the predefined sysId command which would run through the required motions to get a good sysId reading which saves a file to the robot.
My ideal scenario would be we then copy that file back to our computer, we run a single python script which is able to read all the parameters of what this mechanism is and its recorded data and use MatLab to perform the System Identification and all the required calculations. The data I'm looking to get back from it are:
- PID
- Feed forward
- Max current limit (I'm not sure if this one is possible, but basically what I'm looking for is in FRC we have a single battery on the robot, and with brushless motors and swerve drive, managing your power budget has become a crucial painful point in these last couple of years; where you configure and tune everything without any current limits, then you need to go in and slowly start setting these current limits, starting high and lowering them until you find the optimal current limit to performance. Now I think there is probably a good value where for example if you set it to say 60 amps for example, you get 90% of the required perforamnce out of the motor, and setting it to 70 amps gets you 94% of your required performance. (I'm not positive it works this way I'm just guessing based on my experience with tuning these values). So what I'm looking for is maybe something that is interactable based on the System Identification like a slider that shows what your performance would look like at different max currents and gives you a recommended value).

Please let me know if you have any questions. For MatLab, I have the following products installed but if we need any other products that would be useful I can install them as well.
Computer Vision Toolbox
Control System Toolbox
Curve Fitting Toolbox
Model Predicitive Control Toolbox
Navigation Toolbox
Optimization Toolbox
Robotics System Toolbox
Stateflow Toolbox
System Identification Toolbox 

python is available via uv

For this project, I'm really leaning toward a vendor library that we and other teams can install, similar to the QuestNav vendor library, found here:
https://github.com/QuestNav/QuestNav/tree/main/questnav-lib

If you agree that a vendor library is the best way to go, I already created a new folder/repo in the parent folder repos/vendor-lib where you can do the vendor lib implementation, and then do the usage of the vendor-lib in this folder.
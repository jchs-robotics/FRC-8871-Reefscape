package frc.robot.Subsystems;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
    // suh were vuh

    /* set up objects and variables
     * Swerve object
     * telemetry stuff
     * 
     * subsystem functions (constructor)
     * 
     * commands (drive)
    */


    // swerve object
    private final SwerveDrive swerveDrive;

   //Maximum speed of the robot in meters per second, used to limit acceleration.
    public double maximumSpeed = Units.feetToMeters(0); // FIXME placeholder value 
    /*
     * possible values
     * theoretical modules - ~18.8
     * realistic theoretical - 18 
     */

    // swerve constructor
    



    // set up pathplanner



    // drive command


    
}

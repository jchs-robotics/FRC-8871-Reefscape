package frc.robot.Subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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




    // ~~~~~~~~~~ access the json files to construct the swerve ~~~~~~~~~~ //
    public SwerveSubsystem(File directory) {
// directory is the location of the json config files

    // we need to first convert some values into something the robot can read    
        /* 
        *  Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION) // what the following method does
        *  In this case the gear ratio is 18.75 motor revolutions per wheel rotation.
        *  The encoder resolution per motor revolution is 1 per motor revolution. 
        */
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(18.75); 
        /* 
        *  Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION). // what the following method does
        *  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second. 
        *  The gear ratio is 5.36 motor revolutions per wheel rotation.
        *  The encoder resolution per motor revolution is 1 per motor revolution.
        */
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 5.36); 
       
        // print these values
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");




        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        /*
         * Exceptions
         * If an error occurs, the code typically stops to print an error
         * a try and catch method will test the code in the try block, and if an error occurs, will run the catch block.
         * 
         * the throw keyword allows users to create a new custom error
         * in this case its the error of what went wrong with creating the swerve object
         */
        try {
            // creating the swerve drive
        swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, 
                                                                    angleConversionFactor, 
                                                                    driveConversionFactor); // method from last year

        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
        //                                           new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), 
        //                                           Rotation2d.fromDegrees(0))); // includes pose2d (from yagsl)

        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor); // TODO delete this??

        // TODO constructor from 2025 YAGSL


        } catch (Exception e) {
        throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via angle.
        AHRS navx = (AHRS)swerveDrive.swerveDriveConfiguration.imu.getIMU();
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    // setupPathPlanner(); // FIXME will do this later

    } // end of configuring files/swerve


    /*
     * ~~~~~~~~~~ Construct the swerve drive. ~~~~~~~~~~
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */

     // uses constructor from last year
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
    {
       swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed, null); // FIXME figure out the pose2d stuff
        
    }

    // TODO 2025 YAGSL constructor


    

    // ~~~~~~~~~~ set up pathplanner ~~~~~~~~~~ //



    


    // ~~~~~~~~~~ drive code ~~~~~~~~~~ // from YAGSL documentation

  






    // ~~~~~~~~~~ drive commands ~~~~~~~~~~ //

    
} // end of subsystem

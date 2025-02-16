package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meter;



/*
 * Second version of swerve subsystem
 * 
 * swerve tuah (swerve two)
 * drive on that thang
 */
public class SwerveTuahSubsystem extends SubsystemBase {

    // creates swerve drive through the json files
    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerveDrive;


    
    public SwerveTuahSubsystem(File directory) {
        try
        {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                    new Pose2d(new Translation2d(Meter.of(1),
                                                                                                Meter.of(4)),
                                                                                Rotation2d.fromDegrees(0)));
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }
    }

    // returns the swerve drive object
    public SwerveDrive getSwerveDrive() {
       return swerveDrive;
    }


    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    // drive command
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }



}

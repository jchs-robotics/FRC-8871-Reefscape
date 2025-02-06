package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meter;

public class TworveSubsystem extends SubsystemBase {

   
    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerveDrive;

    public TworveSubsystem() {
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


}

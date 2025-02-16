// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveTuahSubsystem;
import swervelib.SwerveInputStream;
import java.io.File;


public class RobotContainer {

  // initialize subsystems and commands here 
  private final SwerveTuahSubsystem swerveTuahSubsystem = new SwerveTuahSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                         "swerve"));

  
  // initialize objects/variables here
  // private final XboxController driveController = new XboxController(0);
  private final CommandXboxController driveController = new CommandXboxController(0);
 





  /*
   * Swerve  code
   */

   // defines the velocity of the robot
   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveTuahSubsystem.getSwerveDrive(),               // get the swerve drive object and get the controller joystick positions
                                                                            () -> driveController.getLeftY() * -1,   // multiplied by -1 because joysticks are inverted
                                                                            () -> driveController.getLeftX() * -1)   //                                                           
                                                                          .withControllerRotationAxis(driveController::getRightX) // rotation axis (sideways movement of joystick)
                                                                          .deadband(OperatorConstants.DEADBAND) // deadband so no stick drift
                                                                          .scaleTranslation(0.8) // TODO learn this
                                                                          .allianceRelativeControl(true); // changes robot movement based on which side

    // defines the angle 
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX, // 
                                                                                              driveController::getRightY)
                                                                                              .headingWhile(true);

    // drive + rotates facing controller pos
    Command driveFieldOrientedDirectAngle = swerveTuahSubsystem.driveFieldOriented(driveDirectAngle);

    // drive + rotate w/ velocity
    Command driveFieldOrientedAngularVelocity = swerveTuahSubsystem.driveFieldOriented(driveAngularVelocity);
    // Command driveOnThatThang = swerveTuahSubsystem.driveFieldOriented(driveAngularVelocity);

  

  /*
   * Robot Container
   * Contains things such as subsystems, commands, and more
   */
  public RobotContainer() {
    configureBindings();
    




    
  }



  private void configureBindings() {
    swerveTuahSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // swerveTuahSubsystem.setDefaultCommand(driveOnThatThang);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


} // end of Robot Container

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
import frc.robot.Subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;
import java.io.File;


public class RobotContainer {

  // initialize subsystems and commands here 
  // private final SwerveTuahSubsystem swerveTuahSubsystem = new SwerveTuahSubsystem(new File(Filesystem.getDeployDirectory(),
  //                                                                                        "swerve"));

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve"));                                                                                        
  
  // initialize objects/variables here
  // private final XboxController driveController = new XboxController(0);
  private final CommandXboxController driveController = new CommandXboxController(0);
 





  /*
   *  Swerve  code
   */

   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),               // get the swerve drive object and get the controller joystick positions
                                                                            () -> driveController.getLeftY() * -1,   // multiplied by -1 because joysticks are inverted
                                                                            () -> driveController.getLeftX() * -1)   //                                                           
                                                                          .withControllerRotationAxis(driveController::getRightX) // rotation axis (sideways movement of joystick)
                                                                          .deadband(OperatorConstants.DEADBAND) // deadband so no stick drift
                                                                          .scaleTranslation(0.8) // TODO learn this
                                                                          .allianceRelativeControl(true); // changes robot movement based on which side

   /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX, // 
                                                                                              driveController::getRightY)
                                                                    .headingWhile(true);

                                                                    
                                                                    




/*
 * Yagsl swerve?
 */

          

/**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
                                                             
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driveController.getLeftY(),
                                                                        () -> -driveController.getLeftX())
                                                                    .withControllerRotationAxis(() -> driveController.getRawAxis(
                                                                        2))
                                                                        .deadband(OperatorConstants.DEADBAND)
                                                                        .scaleTranslation(0.8)
                                                                        .allianceRelativeControl(true);
                                                                        // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                driveController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                driveController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);



  /*
   * Robot Container
   * Contains things such as subsystems, commands, and more
   */
  public RobotContainer() {
    configureBindings();
    



    
    
  }
  
  
  
  private void configureBindings() {
    // drive + rotates facing controller pos
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    
    // drive + rotate w/ velocity
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveOnThatThang = swerveTuahSubsystem.driveFieldOriented(driveAngularVelocity);
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // swerveTuahSubsystem.setDefaultCommand(driveOnThatThang);
    

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


} // end of Robot Container

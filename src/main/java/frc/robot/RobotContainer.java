// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.PivotSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Commands.ElevatorPIDCommand;
import frc.robot.Commands.TurnToCommand;

import swervelib.SwerveInputStream;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;


public class RobotContainer {


private final SendableChooser<Command> autoChooser; // lets us choose our autos

  // initialize subsystems and commands here 
  // private final SwerveTuahSubsystem swerveTuahSubsystem = new SwerveTuahSubsystem(new File(Filesystem.getDeployDirectory(),
  //                                                                                        "swerve"));

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve"));    
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  // initialize objects/variables here
  // private final XboxController driveController = new XboxController(0);
  private final XboxController driveController = new XboxController(0);
  private final XboxController manipulatorController = new XboxController(1);
 





  /*
   *  Swerve  code
   */

   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),               // get the swerve drive object and get the controller joystick positions
                                                                            () -> -driveController.getLeftY(),   // multiplied by -1 because joysticks are inverted
                                                                            () -> -driveController.getLeftX())   //                                                           
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

    // pathplanner commands go here


    drivebase.zeroGyro();

  
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // configures auto chooser
    SmartDashboard.putData("Auto Chooser", autoChooser); // and lets us use smart dashboard to choose auto

    



    
    
  }
  
  
  
  private void configureBindings() {
    // drive + rotates facing controller pos
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    
    // drive + rotate w/ velocity
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Command driveOnThatThang = swerveTuahSubsystem.driveFieldOriented(driveAngularVelocity);
    
    // zero gyro command
    // FIXME placeholder button, switch to like settings button or smth
    new JoystickButton(driveController, 2).onTrue(new InstantCommand(drivebase::zeroGyro));
    
    
    // turn to position command
    
    
    
    
    
    
    
    
    
    
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // swerveTuahSubsystem.setDefaultCommand(driveOnThatThang);
    


    /*
     * Manipulator commands and inputs
     */

     pivotSubsystem.defaultTriggerCommand(manipulatorController.getLeftTriggerAxis(), manipulatorController.getRightTriggerAxis());
     elevatorSubsystem.defaultButtonCommand(manipulatorController.getXButton(), manipulatorController.getAButton());



     /*
      * set elevator positions
      *
      * POV button pos
      * up (L4) - 0
      * right (L3) - 90
      * down (L2) - 180
      * left (stow) - 270
      */

      // stow
      new POVButton(manipulatorController, 180).onTrue(new ElevatorPIDCommand(elevatorSubsystem, 
                                                                                  Constants.ElevatorConstants.STOW_POSITION));
      // L2
      new POVButton(manipulatorController, 270).onTrue(new ElevatorPIDCommand(elevatorSubsystem, 
                                                                                  Constants.ElevatorConstants.L2_POSITION));
      // L3
      new POVButton(manipulatorController, 90).onTrue(new ElevatorPIDCommand(elevatorSubsystem, 
                                                                                  Constants.ElevatorConstants.L3_POSITION));
      // L4
      new POVButton(manipulatorController, 0).onTrue(new ElevatorPIDCommand(elevatorSubsystem, 
                                                                                  Constants.ElevatorConstants.L4_POSITION));





  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return autoChooser.getSelected();
  }


} // end of Robot Container

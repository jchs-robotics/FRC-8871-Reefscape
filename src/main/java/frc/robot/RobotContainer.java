// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Commands.ElevatorJoystickCmd;
import frc.robot.Commands.ElevatorPIDCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.PivotSubsytem;
import frc.robot.Commands.PivotPIDCommand;
import frc.robot.Commands.PivotJoystickCmd;

public class RobotContainer {

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  private final PivotSubsytem pivotSubsytem = new PivotSubsytem();

  private final XboxController xboxController = new XboxController(1);

  private final Joystick joystick1 = new Joystick(0);

  

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick1, 1 ).whileTrue(new ElevatorPIDCommand(elevatorSubsystem, 1.2));
    new JoystickButton(joystick1, 2 ).whileTrue(new ElevatorPIDCommand(elevatorSubsystem, 0));
    new JoystickButton(joystick1, 3 ).whileTrue(new ElevatorJoystickCmd(elevatorSubsystem, 0.5));
    new JoystickButton(joystick1, 4 ).whileTrue(new ElevatorJoystickCmd(elevatorSubsystem, -0.5));
    new JoystickButton(joystick1, 5).whileTrue(new PivotJoystickCmd(pivotSubsytem, 0.5));
    new JoystickButton(joystick1,6).whileTrue(new PivotJoystickCmd(pivotSubsytem, -0.5));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
   
  }
}
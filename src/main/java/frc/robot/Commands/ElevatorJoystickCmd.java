package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorJoystickCmd extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("elevatorJoystickCmd Started!");
    }

    @Override
    public void execute() {
        elevatorSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0);
        System.out.println("elevatorJoystickCmd Ended!");
    }


    @Override
    public boolean isFinished() {
        return false;
    }




}



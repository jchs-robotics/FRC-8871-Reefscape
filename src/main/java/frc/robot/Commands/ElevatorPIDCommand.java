package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;
    private final double endpoint; // used to end the command when reaching positions
    
    
    public ElevatorPIDCommand(ElevatorSubsystem elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(Constants.ElevatorConstants.elevatorP,
                                                Constants.ElevatorConstants.elevatorI,
                                                Constants.ElevatorConstants.elevatorD);
        pidController.setSetpoint(setpoint);
        addRequirements(elevatorSubsystem);

        endpoint = setpoint;
    }


    @Override
    public void initialize() {
        System.out.println("elevator pid command started");
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(elevatorSubsystem.leftEncoder.getPosition());
        elevatorSubsystem.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotors(0);
        System.out.println("elevator PID command ended");
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.bleh(endpoint);
    }
}
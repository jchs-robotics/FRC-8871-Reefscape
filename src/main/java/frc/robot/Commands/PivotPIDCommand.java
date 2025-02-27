package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.PivotSubsystem;

public class PivotPIDCommand extends Command {

    private final PivotSubsystem pivotSubsystem;
    private final PIDController pidController;
    private final double endpoint; // used to end the command when reaching positions
    
    
    public PivotPIDCommand(PivotSubsystem pivotSubsystem, double setpoint) {
        this.pivotSubsystem = pivotSubsystem;
        this.pidController = new PIDController(Constants.PivotConstants.pivotP,
                                                Constants.PivotConstants.pivotI,
                                                Constants.PivotConstants.pivotD);
        pidController.setSetpoint(setpoint);
        addRequirements(pivotSubsystem);

        endpoint = setpoint;
    }


    @Override
    public void initialize() {
        System.out.println("pivot pid command started");
        pidController.reset();
    }

    @Override
    public void execute() {
        double speed = pidController.calculate(pivotSubsystem.leftEncoder);
        pivotSubsystem.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setMotors(0);
        System.out.println("pivot PID command ended");
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.bleh(endpoint);
    }
} 
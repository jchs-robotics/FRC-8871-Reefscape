package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsytem;

public class PivotJoystickCmd extends Command {
    
    private final PivotSubsytem pivotSubsystem;
    private final double speed;

    public PivotJoystickCmd(PivotSubsytem pivotSubsystem, double speed) {
        this.pivotSubsystem = pivotSubsystem;
        this.speed = speed;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("pivotJoystickCmd Started!");
    }

    @Override
    public void execute() {
        pivotSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setMotor(0);
        System.out.println("PivotJoystickCmd Ended!");
    }


    @Override
    public boolean isFinished() {
        return false;
    }




}
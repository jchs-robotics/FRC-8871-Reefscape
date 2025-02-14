package frc.robot.Commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.PivotSubsytem;

public class PivotPIDCommand extends Command {

     private final PivotSubsytem pivotSubsystem;
     private final PIDController pidController;
         
         public PivotPIDCommand(PivotSubsytem pivotsubsystem, double setpoint) {
          this.pivotSubsystem = pivotsubsystem;
          this.pidController = new PIDController(0, 0, 0);
          pidController.setSetpoint(setpoint);
          addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
     System.out.println("PivotPIDCommand ended!");
     pidController.reset();
    }

     @Override
     public void execute() {
          double speed = pidController.calculate(pivotSubsystem.getEncoderMeters());
          pivotSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
     pivotSubsystem.setMotor(0);
     System.out.println("PivotPIDCommand ended!");
    }

    @Override
    public boolean isFinished() {
     return false;
     }

}
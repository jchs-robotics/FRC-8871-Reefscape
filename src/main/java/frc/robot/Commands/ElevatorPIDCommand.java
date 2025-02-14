package frc.robot.Commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends Command {

     private final ElevatorSubsystem elevatorSubsystem;
     private final PIDController pidController;
   

    public ElevatorPIDCommand(ElevatorSubsystem elevatorSubsystem, double setpoint) {
          this.elevatorSubsystem = elevatorSubsystem;
          this.pidController = new PIDController(3, 0, 0.8);
          pidController.setSetpoint(setpoint);
          addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
     System.out.println("ElevatorPIDCommand ended!");
     pidController.reset();
    }

     @Override
     public void execute() {
          double speed = pidController.calculate(elevatorSubsystem.getEncoderMeters());
          elevatorSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
     elevatorSubsystem.setMotor(0);
     System.out.println("ElevatorPIDCommand ended!");
    }

    @Override
    public boolean isFinished() {
     return false;
     }

}




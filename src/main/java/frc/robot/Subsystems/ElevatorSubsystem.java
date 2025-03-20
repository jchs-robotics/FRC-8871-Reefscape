package frc.robot.Subsystems;


import java.net.ContentHandler;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    /*
     * Initialize motor and encoder objects
     */
    private final TalonFX leftElevator = new TalonFX(Constants.ElevatorConstants.LEFT_ELEVATOR_ID);
    private final TalonFX rightElevator = new TalonFX(Constants.ElevatorConstants.RIGHT_ELEVATOR_ID);

    // TODO only need one, can comment whichever one sucks more
    public final double leftEncoder = leftElevator.getPosition().getValueAsDouble();
    public final double rightEncoder = rightElevator.getPosition().getValueAsDouble();

    

    

    public ElevatorSubsystem() {}



    private double leftElevatorPos() {
        return leftElevator.getPosition().getValueAsDouble();
    }
    private double rightElevatorPos() {
        return rightElevator.getPosition().getValueAsDouble();
    }



    @Override
    public void periodic() {
        // telemetry data goes here
        // System.out.println("Left elevator pos: " + leftEncoder);
        // System.out.println("Right elevator pos: " + rightEncoder);
        SmartDashboard.putNumber("Left elevator encoder: ", leftElevatorPos());
        SmartDashboard.putNumber("Right elevator encoder: ", rightElevatorPos());
    }

// FIXME test which side needs to be rotated (do it in tuner)
    public void setMotors(double speed) {
        leftElevator.set(-speed); 
        rightElevator.set(speed);
    }


    // method that stops PID when reaching setpoint
    public boolean bleh(double endpoint) {

        //TODO idk if getPosition() is the correct method for this. we need the encoder ticks to compare

        if (leftEncoder < (endpoint + 5) && leftEncoder > (endpoint - 5)) {
            return true;
        } else {
            return false;
        }



    }


    // default commands
    public void up() {
        setMotors(Constants.ElevatorConstants.manualSpeed);
    }
    public void down() {
        setMotors(-Constants.ElevatorConstants.manualSpeed);
    }
    public void stop() {
        setMotors(0);
    }



    @Override
    public void simulationPeriodic() {

    }



    
}
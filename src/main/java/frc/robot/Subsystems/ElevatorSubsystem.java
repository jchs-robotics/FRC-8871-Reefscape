package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    /*
     * Initialize motor and encoder objects
     */
    private final SparkMax leftElevator = new SparkMax(Constants.ElevatorConstants.LEFT_ELEVATOR_ID, MotorType.kBrushless);
    private final SparkMax rightElevator = new SparkMax(Constants.ElevatorConstants.RIGHT_ELEVATOR_ID, MotorType.kBrushless);

    // TODO only need one, can comment whichever one sucks more
    public final RelativeEncoder leftEncoder = leftElevator.getEncoder();
    public final RelativeEncoder rightEncoder = rightElevator.getEncoder();

    

    public ElevatorSubsystem() {}

    @Override
    public void periodic() {
        // telemetry data goes here
    } 


    public void setMotors(double speed) {
        leftElevator.set(speed);
        rightElevator.set(speed);
    }


    // method that stops PID when reaching setpoint
    public boolean bleh(double endpoint) {

        //TODO idk if getPosition() is the correct method for this. we need the encoder ticks to compare

        if (leftEncoder.getPosition() < (endpoint + 5) && leftEncoder.getPosition() > (endpoint - 5)) {
            return true;
        } else {
            return false;
        }



    }


    // default command for trigger inputs
    /* public void defaultTriggerCommand(double leftJoystick, double rightJoystick) {
        if (leftJoystick < -0.05) {
            leftElevator.set(leftJoystick);
            rightElevator.set(leftJoystick);
        } else if (rightJoystick < -0.05) {
            leftElevator.set(rightJoystick);
            rightElevator.set(rightJoystick);
        } else {
            leftElevator.set(0);
            rightElevator.set(0);
        }
    } */

    // TODO default command for button inputs
    public void defaultButtonCommand(boolean upInput, boolean downInput) {
        if (upInput) {
            leftElevator.set(Constants.ElevatorConstants.manualSpeed);
            rightElevator.set(Constants.ElevatorConstants.manualSpeed);
        } else if (downInput) {
            leftElevator.set(-Constants.ElevatorConstants.manualSpeed);
            rightElevator.set(-Constants.ElevatorConstants.manualSpeed);
        } else {
            leftElevator.set(0);
            rightElevator.set(0);
        }
    }


    @Override
    public void simulationPeriodic() {

    }



    
} 
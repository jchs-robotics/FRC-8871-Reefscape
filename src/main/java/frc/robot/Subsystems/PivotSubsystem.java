package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

    /*
     * Initialize motor and encoder objects
     */
    private final SparkMax leftPivot = new SparkMax(Constants.PivotConstants.LEFT_PIVOT_ID, MotorType.kBrushless);
    private final SparkMax rightPivot = new SparkMax(Constants.PivotConstants.RIGHT_PIVOT_ID, MotorType.kBrushless);

    // TODO only need one, can comment whichever one sucks more
    public final RelativeEncoder leftEncoder = leftPivot.getEncoder();
    public final RelativeEncoder rightEncoder = rightPivot.getEncoder();

    public PivotSubsystem() {}

    @Override
    public void periodic() {
        // telemetry data goes here
    } 


    public void setMotors(double speed) {
        leftPivot.set(speed);
        rightPivot.set(speed);
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
    public void defaultTriggerCommand(double leftTrigger, double rightTrigger) {
        if (leftTrigger > 0.05) {
            leftPivot.set(leftTrigger);
            rightPivot.set(leftTrigger);
        } else if (rightTrigger > 0.05) {
            leftPivot.set(rightTrigger);
            rightPivot.set(rightTrigger);
        } else {
            leftPivot.set(0);
            rightPivot.set(0);
        }
    }


    @Override
    public void simulationPeriodic() {

    }



    
}

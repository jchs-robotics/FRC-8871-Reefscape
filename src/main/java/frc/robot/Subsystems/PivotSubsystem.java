package frc.robot.Subsystems;


import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

    /*
     * Initialize motor and encoder objects
     */
    private final TalonFX leftPivot = new TalonFX(Constants.PivotConstants.LEFT_PIVOT_ID);
    private final TalonFX rightPivot = new TalonFX(Constants.PivotConstants.RIGHT_PIVOT_ID);

    

    // TODO only need one, can comment whichever one sucks more
    public final double leftEncoder = leftPivot.getPosition().getValueAsDouble();
    public final double rightEncoder = rightPivot.getPosition().getValueAsDouble();

    public PivotSubsystem() {}


    public double leftPivotPos() {
        return leftPivot.getPosition().getValueAsDouble();
    }
    private double rightPivotPos() {
        return rightPivot.getPosition().getValueAsDouble();
    }



    @Override
    public void periodic() {
        // telemetry data goes here

        // System.out.println("left pivot encoder: " + leftEncoder);
        
        
        SmartDashboard.putNumber("Left pivot encoder: ", leftPivotPos());
        SmartDashboard.putNumber("Right pivot encoder: ", rightPivotPos());
    } 


    // FIXME test which side needs to be rotated (do it in tuner)
    public void setMotors(double speed) {
        leftPivot.set(-speed);
        rightPivot.set(speed);
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


    public void manualControl(double leftTrigger, double rightTrigger) {
        setMotors(rightTrigger - leftTrigger);
    }



    public void up() {
        setMotors(Constants.PivotConstants.manualSpeed);
    }
    public void down() {
        setMotors(-Constants.PivotConstants.manualSpeed);
    }
    public void stop() {
        setMotors(0);
    }



    @Override
    public void simulationPeriodic() {

    }



    
}

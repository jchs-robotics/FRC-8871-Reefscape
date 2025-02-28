package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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


    private double leftPivotPos() {
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


// FIXME test which side needs to be rotated (do it in tuner)
    // default command for trigger inputs
    public Command manual(double leftTrigger, double rightTrigger) {
        
        return run(() -> {
            // if (leftTrigger > 0.05) {
            //     leftPivot.set(-leftTrigger);
            //     rightPivot.set(leftTrigger);
            // } else if (rightTrigger > 0.05) {
            //     leftPivot.set(rightTrigger);
            //     rightPivot.set(-rightTrigger);
            // } else {
            //     leftPivot.set(0);
            //     rightPivot.set(0);
            // }

            setMotors(rightTrigger - leftTrigger);
        });

    }

    public void manualControl(double leftTrigger, double rightTrigger) {
        setMotors(rightTrigger - leftTrigger);
    }

    

    public Command manual(double joystick) {
        return run(() -> {
            setMotors(-joystick);
        });
    }

    public void up() {
        setMotors(-Constants.PivotConstants.manualSpeed);
    }
    public void down() {
        setMotors(Constants.PivotConstants.manualSpeed);
    }
    public void stop() {
        setMotors(0);
    }

    // public void manualPivot(double leftTrigger, double rightTrigger) {
    //     // setMotors(rightTrigger - leftTrigger);

    //     if (leftTrigger > 0.05) {
    //                     leftPivot.set(-leftTrigger);
    //                     rightPivot.set(leftTrigger);
    //                 } else if (rightTrigger > 0.05) {
    //                     leftPivot.set(rightTrigger);
    //                     rightPivot.set(-rightTrigger);
    //                 } else {
    //                     leftPivot.set(0);
    //                     rightPivot.set(0);
    //                 }
    // }


    @Override
    public void simulationPeriodic() {

    }



    
}

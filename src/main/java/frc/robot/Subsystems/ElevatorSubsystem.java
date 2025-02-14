package frc.robot.Subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax elevatorMotor = new SparkMax(21, MotorType.kBrushless);
    private final Encoder encoder = new Encoder(4, 5);
     private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;
    
     public double getEncoderMeters() {
        return encoder.get() * kEncoderTick2Meter;
     }  

        public ElevatorSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setMotor(double speed) {
        elevatorMotor.set(speed);
    }
}


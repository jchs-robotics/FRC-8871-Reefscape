package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsytem extends SubsystemBase {
    
    private final Spark pivotSubsystem = new Spark(13);
    private final Encoder encoder = new Encoder(4, 5);
     private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;
    
     public double getEncoderMeters() {
        return encoder.get() * kEncoderTick2Meter;
     }  

        public void PivotSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot encoder value", getEncoderMeters());
    }

    public void setMotor(double speed) {
        pivotSubsystem.set(speed);
    }
}



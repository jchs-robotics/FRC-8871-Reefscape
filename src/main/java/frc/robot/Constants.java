package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

public final class Constants {
    
    // constants for pathplanner
    public static class PathPlannerConstants {
            public static RobotConfig config; 

            // {
            // try{
            //      config = RobotConfig.fromGUISettings();
            //   } catch (Exception e) {
            //     // Handle exception as needed
            //     e.printStackTrace();
            //   }
            // }

            // public static RobotConfig config = RobotConfig.fromGUISettings();
    }

    // constants for a drive controller
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double DEADBAND = 0.05;
    }

    public static final  double MAX_SPEED = Units.feetToMeters(4.5);

    public class ElevatorConstants {
        public static int LEFT_ELEVATOR_ID = 5;
        public static int RIGHT_ELEVATOR_ID = 6;

        public static double elevatorP = 0;
        public static double elevatorI = 0;
        public static double elevatorD = 0;

        public static double manualSpeed = 0.9;
    }

    public class PivotConstants {
        public static int LEFT_PIVOT_ID = 3;
        public static int RIGHT_PIVOT_ID = 4;

        public static double pivotP = 0;
        public static double pivotI = 0;
        public static double pivotD = 0;
    }

    
}

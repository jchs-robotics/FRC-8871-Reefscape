package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;


// TODO fix constants
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

    public static final  double MAX_SPEED = Units.feetToMeters(18);

    public class ElevatorConstants {
        public static int LEFT_ELEVATOR_ID = 5;
        public static int RIGHT_ELEVATOR_ID = 6;

                                                // last year values
        public static double elevatorP = 0.1;   // 0.7
        public static double elevatorI = 0;     // 0.01
        public static double elevatorD = 0;     // 0.01

        // TODO
        public static double STOW_POSITION = 5;
        public static double L2_POSITION = 0;
        public static double L3_POSITION = 0;
        public static double L4_POSITION = 0;

        public static double manualSpeed = 0.2;
    }

    public class PivotConstants {
        public static int LEFT_PIVOT_ID = 3;
        public static int RIGHT_PIVOT_ID = 4;

                                            // last year values
        public static double pivotP = 0.1;  // 0.1 // 0.28
        public static double pivotI = 0;    // 0.0
        public static double pivotD = 0;    // 0.009

        // TODO
        public static double STOW_POSITION = 5;
        public static double INTAKE_POSITION = 0;
        public static double SCORE_POSITION = 0;
        
        public static double manualSpeed = 0.6;
    }

    
}
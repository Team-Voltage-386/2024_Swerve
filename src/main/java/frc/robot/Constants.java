package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class Constants {
    public static class Swerve {
        // PIDs for the Swerve motors
        // Steering Motors
        public static final int kSteeringFrontLeftPWM = 0;
        public static final int kSteeringFrontRightPWM = 0;
        public static final int kSteeringBackLeftPWM = 0;
        public static final int kSteeringBackRightPWM = 0;

        // Driving Motors
        public static final int kDrivingFrontRightPWM = 0;
        public static final int kDrivingFrontLeftPWM = 0;
        public static final int kDrivingBackRightPWM = 0;
        public static final int kDrivingBackLeftPWM = 0;

        // Driving PID
        private static final int kDriveP = 0;
        private static final int kDriveI = 0;
        private static final int kDriveD = 0;
        private static final int kDriveTollerance = 0;
        public static final PIDController DrivePID = new PIDController(kDriveP, kDriveI, kDriveD);

        // Steering PID
        private static final int kSteerP = 0;
        private static final int kSteerI = 0;
        private static final int kSteerD = 0;
        public static final PIDController SteerPID = new PIDController(kSteerP, kSteerI, kSteerD);

        // The Length and Width of the Chasis from one swerve axel to the next.
        public static final double kchassisLength = 0; // From either back axel to the front axel on the same side.
        public static final double kchassisWidth = 0; // From either left axel to the right axel on the same side.

    }
}

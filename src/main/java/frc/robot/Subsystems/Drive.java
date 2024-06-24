package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class Drive extends SubsystemBase {

    // Converts input from the controllers to directios for each swerve drive
    // module.
    public void drive(
            double x1, // Forward (Positive) and Backward (Negative) Strafing
            double y1, // Right (Positive) and Left (Negative) Strafing
            double x2 // Clockwise (Positive) and Counterclockwise (Negative) Rotation directions
    ) {
        double r = Math.sqrt( /* the Radious of the Chassis */
                (Swerve.kchassisLength * Swerve.kchassisLength) + (Swerve.kchassisWidth * Swerve.kchassisWidth));
        y1 *= -1;

        // Offsets for the speed and steering
        double a = x1 - x2 * (Swerve.kchassisLength / r);
        double b = x1 + x2 * (Swerve.kchassisLength / r);
        double c = y1 - x2 * (Swerve.kchassisWidth / r);
        double d = y1 + x2 * (Swerve.kchassisWidth / r);

        // The speed of each module
        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        // The angle of each swerve module
        double backRightAngle = Math.atan2(a, d) / Math.PI;
        double backLeftAngle = Math.atan2(a, c) / Math.PI;
        double frontRightAngle = Math.atan2(b, d) / Math.PI;
        double frontLeftAngle = Math.atan2(b, c) / Math.PI;
    }

}

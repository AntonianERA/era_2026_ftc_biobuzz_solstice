package org.firstinspires.ftc.teamcode.driveTrain;

/**
 * Field-centric (geocentric) mecanum drive math.
 * Call from any OpMode: GeocentricDrive.drive(driveTrain, x, y, rx);
 */
public class geocentricDrive {

    private geocentricDrive() {} // static-only utility class

    /**
     * @param dt the robot's DriveTrain
     * @param x  strafe input  (gamepad1.left_stick_x)
     * @param y  forward input (-gamepad1.left_stick_y)
     * @param rx turn input    (gamepad1.right_stick_x)
     */
    public static void drive(driveTrain dt, double x, double y, double rx) {
        x = x * 1.1; // counteract imperfect strafing

        // Normalize so no motor power exceeds 1 while keeping ratios
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        dt.setPowers(
                (y + x + rx) / denom, // front left
                (y - x + rx) / denom, // back left
                (y - x - rx) / denom, // front right
                (y + x - rx) / denom  // back right
        );

    }
}
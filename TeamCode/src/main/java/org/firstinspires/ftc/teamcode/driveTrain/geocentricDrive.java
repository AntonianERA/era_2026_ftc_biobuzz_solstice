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
        double heading = dt.getHeading();

        // Rotate the joystick vector by -heading so "forward" stays field-forward
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Normalize so no motor power exceeds 1 while keeping ratios
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        dt.setPowers(
                (rotY + rotX + rx) / denom, // front left
                (rotY - rotX + rx) / denom, // back left
                (rotY - rotX - rx) / denom, // front right
                (rotY + rotX - rx) / denom  // back right
        );
    }
}
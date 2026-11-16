package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Field-centric mecanum drive using the goBILDA Pinpoint Odometry Computer.
 * This is NOT an OpMode - create one inside your own OpMode and call it:
 *
 *   FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);   // in init
 *   drive.drive(gamepad1);                                          // every loop
 *
 * Robot configuration names expected:
 *   Motors:   "frontLeft", "frontRight", "backLeft", "backRight"
 *   I2C:      "pinpoint"  (goBILDA Pinpoint Odometry Computer)
 */
public class FieldCentricDrive {

    // ---------------- TUNE THESE FOR YOUR ROBOT ----------------

    // Pod positions relative to the robot's center of rotation, in mm.
    // X_POD_OFFSET: how far LEFT (+) or right (-) the forward-tracking (X) pod is.
    // Y_POD_OFFSET: how far FORWARD (+) or back (-) the strafe-tracking (Y) pod is.
    private static final double X_POD_OFFSET_MM = -84.0;
    private static final double Y_POD_OFFSET_MM = -168.0;

    // goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // X pod should count UP moving forward, Y pod should count UP moving left.
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private static final double STRAFE_CORRECTION = 1.1;
    private static final double SLOW_MODE_SCALE   = 0.35;

    // ------------------------------------------------------------

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final GoBildaPinpointDriver pinpoint;

    private double headingOffsetRad = 0.0;
    private double heading = 0.0;

    /**
     * Call once during init. THE ROBOT MUST BE STILL - this calibrates the
     * Pinpoint's IMU (~0.25 s) and zeroes its position.
     */
    public FieldCentricDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Positive power should spin every wheel "forward". Flip any that don't.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU();
    }

    /**
     * Easiest way to drive: call once per loop with the driver's gamepad.
     *   Left stick - drive/strafe (field-relative), Right stick X - rotate,
     *   Right trigger - slow mode, Options - re-zero heading.
     */
    public void drive(Gamepad gamepad) {
        if (gamepad.options) {
            pinpoint.update();
            resetHeading();
        }
        double scale = gamepad.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
        drive(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, scale);
    }

    /**
     * Drive with raw values, if you want your own control scheme.
     * Call once per loop (this is what reads the Pinpoint).
     *
     * @param forward  +1 = away from the driver
     * @param strafe   +1 = to the driver's right
     * @param turn     +1 = clockwise
     * @param scale    overall speed multiplier, 0 to 1
     */
    public void drive(double forward, double strafe, double turn, double scale) {
        pinpoint.update(); // fresh data, once per loop
        heading = pinpoint.getHeading(AngleUnit.RADIANS) - headingOffsetRad;

        // Rotate the joystick vector by the opposite of the robot's heading.
        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        rotX *= STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
        frontLeft.setPower((rotY + rotX + turn) / denominator * scale);
        backLeft.setPower((rotY - rotX + turn) / denominator * scale);
        frontRight.setPower((rotY - rotX - turn) / denominator * scale);
        backRight.setPower((rotY + rotX - turn) / denominator * scale);
    }

    /** Makes the direction the robot is facing right now the new "forward". */
    public void resetHeading() {
        headingOffsetRad = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    /** Heading in degrees, CCW positive, relative to the last re-zero. */
    public double getHeadingDegrees() { return Math.toDegrees(heading); }
    public double getX(DistanceUnit unit) { return pinpoint.getPosX(unit); }
    public double getY(DistanceUnit unit) { return pinpoint.getPosY(unit); }

    /** Optional: adds pose info to telemetry (you still call telemetry.update()). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("X (in)", "%.1f", getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", "%.1f", getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.1f", getHeadingDegrees());
    }
}
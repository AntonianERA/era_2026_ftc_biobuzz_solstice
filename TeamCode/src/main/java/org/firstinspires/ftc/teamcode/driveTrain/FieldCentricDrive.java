package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Field-centric mecanum drive using the goBILDA Pinpoint, with heading hold and
 * strafe drift compensation (values tuned with TestOpMode_FieldCentricDriveTest).
 *
 * This is NOT an OpMode. Hardware + actions only - the buttons live in RobotControl:
 *   FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);  // init, robot still
 *   drive.drive(forward, strafe, turn, scale);                     // once per loop
 *
 * Config names: "frontLeft", "frontRight", "backLeft", "backRight", "pinpoint"
 */
public class FieldCentricDrive {

    // ================= TUNED VALUES =================
    public static double HEADING_KP       = 3.0;    // heading hold strength
    public static double RIGHT_DRIFT_COMP = 0.13;   // strafe right: + = was creeping forward
    public static double LEFT_DRIFT_COMP  = -0.13;  // strafe left:  + = was creeping forward
    // ================================================

    // ----- Pinpoint / pods -----
    private static final double X_POD_OFFSET_MM = -84.0;   // only affects X/Y, not driving
    private static final double Y_POD_OFFSET_MM = -168.0;
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    private static final double STRAFE_CORRECTION = 1.1;
    private static final double MAX_CORRECTION    = 0.5;
    private static final double TURN_DEADBAND     = 0.05;
    private static final double SETTLE_TIME_SEC   = 0.25;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final GoBildaPinpointDriver pinpoint;

    private double headingOffsetRad = 0.0;
    private double heading = 0.0;
    private double targetHeading = 0.0;
    private boolean holding = false;
    private boolean headingHoldEnabled = true;
    private final ElapsedTime sinceTurn = new ElapsedTime();

    /** Call once during init. THE ROBOT MUST BE STILL (calibrates the Pinpoint IMU). */
    public FieldCentricDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Positive power = every wheel spins forward (verified on the robot).
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
     * Drive with raw values. Call once per loop (this reads the Pinpoint).
     * @param forward +1 = away from driver   @param strafe +1 = driver's right
     * @param turn    +1 = clockwise          @param scale  speed multiplier 0..1
     */
    public void drive(double forward, double strafe, double turn, double scale) {
        pinpoint.update();
        heading = pinpoint.getHeading(AngleUnit.RADIANS) - headingOffsetRad;

        forward *= scale;
        strafe  *= scale;
        turn    *= scale;

        // ----- Heading hold: stops the nose rotating while driving -----
        boolean driverTurning = Math.abs(turn) > TURN_DEADBAND;
        boolean translating   = Math.abs(forward) + Math.abs(strafe) > 0.05;

        if (driverTurning || !headingHoldEnabled) {
            holding = false;
            sinceTurn.reset();
        } else if (!holding) {
            targetHeading = heading;
            if (sinceTurn.seconds() > SETTLE_TIME_SEC) holding = true;
        }

        if (holding && translating) {
            double error = angleWrap(heading - targetHeading);   // CCW drift -> turn CW
            turn = Range.clip(HEADING_KP * error, -MAX_CORRECTION, MAX_CORRECTION);
        } else if (holding) {
            targetHeading = heading;
        }

        // ----- Field-centric rotation -----
        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

        // ----- Strafe drift comp (robot-relative, per side) -----
        double comp = rotX > 0 ? RIGHT_DRIFT_COMP : LEFT_DRIFT_COMP;
        rotY -= comp * Math.abs(rotX);
        rotX *= STRAFE_CORRECTION;

        // ----- Mecanum mixing -----
        double d = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
        frontLeft.setPower((rotY + rotX + turn) / d);
        backLeft.setPower((rotY - rotX + turn) / d);
        frontRight.setPower((rotY - rotX - turn) / d);
        backRight.setPower((rotY + rotX - turn) / d);
    }

    /** Makes the direction the robot faces right now the new "forward". */
    public void resetHeading() {
        headingOffsetRad = pinpoint.getHeading(AngleUnit.RADIANS);
        heading = 0.0;
        targetHeading = 0.0;
    }

    public void setHeadingHoldEnabled(boolean enabled) {
        headingHoldEnabled = enabled;
        holding = false;
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

    /** Adds pose info to telemetry (you still call telemetry.update()). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Heading (deg)", "%.1f", getHeadingDegrees());
        telemetry.addData("X / Y (in)", "%.1f / %.1f", getX(DistanceUnit.INCH), getY(DistanceUnit.INCH));
    }

    private static double angleWrap(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }
}

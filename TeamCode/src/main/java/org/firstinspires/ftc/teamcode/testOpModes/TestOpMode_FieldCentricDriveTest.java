package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Standalone field-centric mecanum TeleOp with live tuning for strafe drift.
 *
 * DRIVING (gamepad 1):
 *   Left stick      - drive / strafe (field-centric)
 *   Right stick X   - turn
 *   Right trigger   - slow mode (hold)
 *   Options         - re-zero heading (robot faces away from you, then press)
 *
 * TUNING (gamepad 1) - values show on the Driver Station:
 *   DPAD UP / DOWN    - heading hold strength (kP): fixes the nose ROTATING
 *   DPAD RIGHT / LEFT - strafe-RIGHT drift: + if strafing right creeps FORWARD
 *   X / B             - strafe-LEFT drift:  + if strafing left creeps FORWARD
 *   A                 - heading hold ON / OFF
 *
 * When it drives straight, copy the three numbers into the defaults below.
 */
@TeleOp(name = "Field Centric (Tunable)", group = "Drive")
public class TestOpMode_FieldCentricDriveTest extends LinearOpMode {

    // ================= TUNED VALUES - copy your final numbers here =================
    private double headingKp       = 3.0;   // heading hold strength
    private double rightDriftComp  = 0.13;  // strafe right: + = it was creeping forward
    private double leftDriftComp   = -0.13; // strafe left:  + = it was creeping forward
    // ==============================================================================

    private static final double KP_STEP    = 0.25;
    private static final double DRIFT_STEP = 0.01;   // small steps for a slight curve

    // ----- Robot settings (from your tests) -----
    private static final double X_POD_OFFSET_MM = -84.0;   // only matters for X/Y, not driving
    private static final double Y_POD_OFFSET_MM = -168.0;
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    private static final double STRAFE_CORRECTION = 1.1;
    private static final double SLOW_MODE_SCALE   = 0.35;

    private static final double MAX_CORRECTION  = 0.5;
    private static final double TURN_DEADBAND   = 0.05;
    private static final double SETTLE_TIME_SEC = 0.25;

    @Override
    public void runOpMode() {
        // ----- Motors (directions that worked in DriveTest) -----
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ----- Pinpoint -----
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU(); // robot must be still

        telemetry.addLine("Ready. Keep robot still.");
        telemetry.addLine("Face robot AWAY from you, then press play.");
        telemetry.update();

        waitForStart();

        double headingOffset = 0.0;
        double targetHeading = 0.0;
        boolean holding = false;
        boolean holdEnabled = true;
        ElapsedTime sinceTurn = new ElapsedTime();

        boolean pA = false, pUp = false, pDown = false, pL = false, pR = false, pX = false, pB = false;

        while (opModeIsActive()) {
            pinpoint.update(); // once per loop

            // ----- Tuning buttons (on press) -----
            if (gamepad1.a && !pA)              { holdEnabled = !holdEnabled; holding = false; }
            if (gamepad1.dpad_up && !pUp)       headingKp += KP_STEP;
            if (gamepad1.dpad_down && !pDown)   headingKp = Math.max(0, headingKp - KP_STEP);
            if (gamepad1.dpad_right && !pR)     rightDriftComp += DRIFT_STEP;
            if (gamepad1.dpad_left && !pL)      rightDriftComp -= DRIFT_STEP;
            if (gamepad1.x && !pX)              leftDriftComp += DRIFT_STEP;
            if (gamepad1.b && !pB)              leftDriftComp -= DRIFT_STEP;
            pA = gamepad1.a; pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
            pL = gamepad1.dpad_left; pR = gamepad1.dpad_right; pX = gamepad1.x; pB = gamepad1.b;

            if (gamepad1.options) {
                headingOffset = pinpoint.getHeading(AngleUnit.RADIANS);
                targetHeading = 0.0;
            }
            double heading = pinpoint.getHeading(AngleUnit.RADIANS) - headingOffset;

            // ----- Driver input -----
            double scale   = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
            double forward = -gamepad1.left_stick_y * scale;
            double strafe  =  gamepad1.left_stick_x * scale;
            double turn    =  gamepad1.right_stick_x * scale;

            // ----- Heading hold (stops the nose rotating) -----
            boolean driverTurning = Math.abs(turn) > TURN_DEADBAND;
            boolean translating   = Math.abs(forward) + Math.abs(strafe) > 0.05;
            double error = 0.0;

            if (driverTurning || !holdEnabled) {
                holding = false;
                sinceTurn.reset();
            } else if (!holding) {
                targetHeading = heading;
                if (sinceTurn.seconds() > SETTLE_TIME_SEC) holding = true;
            }

            if (holding && translating) {
                error = angleWrap(heading - targetHeading);      // CCW drift -> turn CW
                turn = Range.clip(headingKp * error, -MAX_CORRECTION, MAX_CORRECTION);
            } else if (holding) {
                targetHeading = heading;
            }

            // ----- Field-centric rotation -----
            double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

            // ----- Strafe drift comp (robot-relative, separate for each side) -----
            // Positive comp = that strafe direction was creeping forward, so push back a bit.
            double comp = rotX > 0 ? rightDriftComp : leftDriftComp;
            rotY -= comp * Math.abs(rotX);

            rotX *= STRAFE_CORRECTION;

            // ----- Mecanum mixing -----
            double d = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
            frontLeft.setPower((rotY + rotX + turn) / d);
            backLeft.setPower((rotY - rotX + turn) / d);
            frontRight.setPower((rotY - rotX - turn) / d);
            backRight.setPower((rotY + rotX - turn) / d);

            // ----- Telemetry -----
            telemetry.addLine("=== COPY THESE WHEN IT DRIVES STRAIGHT ===");
            telemetry.addData("headingKp      (DPAD U/D)", "%.2f", headingKp);
            telemetry.addData("rightDriftComp (DPAD R/L)", "%.2f", rightDriftComp);
            telemetry.addData("leftDriftComp  (X / B)", "%.2f", leftDriftComp);
            telemetry.addData("Heading hold   (A)", holdEnabled ? "ON" : "OFF");
            telemetry.addLine();
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Hold error (deg)", "%.1f", Math.toDegrees(error));
            telemetry.addLine();
            telemetry.addLine("Nose rotates while strafing -> DPAD UP");
            telemetry.addLine("Nose wobbles/shakes        -> DPAD DOWN");
            telemetry.addLine("Strafe RIGHT creeps fwd -> DPAD RIGHT, back -> DPAD LEFT");
            telemetry.addLine("Strafe LEFT  creeps fwd -> X,          back -> B");
            telemetry.update();
        }
    }

    /** Wraps an angle to -PI..PI. */
    private static double angleWrap(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }
}
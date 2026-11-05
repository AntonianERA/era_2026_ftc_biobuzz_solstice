package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * RPM Tuner for a goBILDA Yellow Jacket 5203 1:1 (6000 RPM, 28 ticks/rev)
 *
 * Controls (gamepad1):
 *   A             - start / stop the motor
 *   D-pad up/down - +/- 100 RPM (coarse)
 *   Right/Left bumper - +/- 10 RPM (fine)
 *   Y             - reverse direction
 *   B             - reset target to 0 and stop
 */
@TeleOp(name = "RPM Tuner", group = "Test")
public class TestOpMode_rpmTuner extends LinearOpMode {

    // ---- Change these to match your setup ----
    static final String MOTOR_NAME = "outake";

    private DcMotor intake;
    private DcMotor intake2;// name in your robot configuration
    static final double TICKS_PER_REV = 28.0;   // 5203 1:1 bare motor
    static final double MAX_RPM = 6000.0;       // rated free speed
    static final double COARSE_STEP = 100.0;
    static final double FINE_STEP = 10.0;
    // -------------------------------------------

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");// let a flywheel coast down

        double targetRPM = 1000;
        boolean running = false;
        boolean reversed = false;

        // previous button states for edge detection
        boolean prevA = false, prevB = false, prevY = false;
        boolean prevUp = false, prevDown = false, prevRB = false, prevLB = false;

        // peak tracking
        double minRPMSeen = Double.MAX_VALUE;
        double maxRPMSeen = 0;

        telemetry.addLine("Ready. Press START, then A to spin up.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- input handling (edge-triggered) ---
            if (gamepad1.a && !prevA) {
                running = !running;
                minRPMSeen = Double.MAX_VALUE;
                maxRPMSeen = 0;
            }
            if (gamepad1.b && !prevB) {
                running = false;
                targetRPM = 0;
            }
            if (gamepad1.y ) {
                intake.setPower(1);
                intake2.setPower(1);
            }else{
                intake.setPower(0);
                intake2.setPower(0);
            }
            if (gamepad1.dpad_up && !prevUp)       targetRPM += COARSE_STEP;
            if (gamepad1.dpad_down && !prevDown)   targetRPM -= COARSE_STEP;
            if (gamepad1.right_bumper && !prevRB)  targetRPM += FINE_STEP;
            if (gamepad1.left_bumper && !prevLB)   targetRPM -= FINE_STEP;

            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevY = gamepad1.y;
            prevUp = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;
            prevRB = gamepad1.right_bumper;
            prevLB = gamepad1.left_bumper;

            // --- motor command ---
            double targetTicksPerSec = targetRPM * TICKS_PER_REV / 60.0;
            motor.setVelocity(running ? targetTicksPerSec : 0);

            // --- measurement ---
            double actualTicksPerSec = motor.getVelocity();
            double actualRPM = Math.abs(actualTicksPerSec) * 60.0 / TICKS_PER_REV;
            double error = targetRPM - actualRPM;

            if (running) {
                minRPMSeen = Math.min(minRPMSeen, actualRPM);
                maxRPMSeen = Math.max(maxRPMSeen, actualRPM);
            }

            // --- telemetry ---
            telemetry.addData("Status", running ? "RUNNING" : "STOPPED");
            telemetry.addData("Direction", reversed ? "Reverse" : "Forward");
            telemetry.addLine();
            telemetry.addData("Target RPM", "%.0f  (%.0f%% of max)", targetRPM, 100 * targetRPM / MAX_RPM);
            telemetry.addData("Actual RPM", "%.0f", actualRPM);
            telemetry.addData("Error RPM", "%.0f", running ? error : 0);
            telemetry.addData("Min / Max seen", running && maxRPMSeen > 0
                    ? String.format("%.0f / %.0f", minRPMSeen, maxRPMSeen) : "-");
            telemetry.addLine();
            telemetry.addData("Target ticks/s", "%.1f", targetTicksPerSec);
            telemetry.addData("Actual ticks/s", "%.1f", actualTicksPerSec);
            telemetry.addData("Current (A)", "%.2f", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Battery (V)", "%.2f",
                    hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addLine();
            telemetry.addLine("A start/stop | Dpad ±100 | Bumpers ±10 | Y reverse | B reset");
            telemetry.update();
        }

        motor.setVelocity(0);
    }
}
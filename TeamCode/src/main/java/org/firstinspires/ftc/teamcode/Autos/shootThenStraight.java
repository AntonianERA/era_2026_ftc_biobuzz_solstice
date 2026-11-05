package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Time-based autonomous: spin up outake, kick, reset kicker, then drive forward.
@Autonomous(name = "shoot then drive")
public class shootThenStraight extends LinearOpMode {

    // ---- Tuning values ----
    static final double OUTAKE_RPM      = 2450;
    static final double TICKS_PER_REV   = 28;    // bare motor encoder
    static final double KICKER_REST     = 0.0;   // same as TeleOp
    static final double KICKER_KICK     = 0.15;   // same as TeleOp
    static final double SPIN_UP_TIME    = 1.0;   // seconds
    static final double KICK_HOLD_TIME  = 0.5;   // seconds

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotorEx outake;
    private Servo kicker;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ---- INIT ----
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outake = hardwareMap.get(DcMotorEx.class, "outake");
        outake.setDirection(DcMotorSimple.Direction.REVERSE);
        outake.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // needed for setVelocity

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_REST);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // ---- AUTO STEPS ----
        shoot();
        drive(0.25, 0, 0, 1.0);   // forward at 0.25 for 1 second

        stopDrive();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    // Spin up the outake, kick, wait, reset kicker, stop outake.
    private void shoot() {
        double ticksPerSecond = (OUTAKE_RPM * TICKS_PER_REV) / 60.0;

        outake.setVelocity(ticksPerSecond);
        waitFor(SPIN_UP_TIME, "Spinning up");

        kicker.setPosition(KICKER_KICK);
        waitFor(KICK_HOLD_TIME, "Kicking");

        kicker.setPosition(KICKER_REST);
        outake.setVelocity(0);
    }

    // Waits for a set time while showing telemetry. Stops early if Stop is pressed.
    private void waitFor(double seconds, String label) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Step", label);
            telemetry.addData("Time", "%.2f / %.2f s", runtime.seconds(), seconds);
            telemetry.addData("Outake vel", "%.0f ticks/s", outake.getVelocity());
            telemetry.update();
        }
    }

    // Runs the drivetrain with the same mecanum math as TeleOp for a set time, then stops.
    private void drive(double axial, double lateral, double yaw, double seconds) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        waitFor(seconds, "Driving");

        stopDrive();
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Time-based autonomous. Drives forward for 1 second at 25% power.
@Autonomous(name = "just drive")
public class straight extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ---- INIT ----
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");

        // Same directions as the TeleOp so "positive" = forward
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // ---- AUTO STEPS ----
        // drive(axial, lateral, yaw, seconds)
        //   axial   + = forward,   - = backward
        //   lateral + = strafe (same sign as TeleOp lateral)
        //   yaw     + = turn       (same sign as TeleOp yaw)
        drive(0.25, 0, 0, 1.0);   // forward at 0.25 for 1 second

        stopDrive();
        telemetry.addData("Status", "Done");
        telemetry.update();
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

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Step time", "%.2f / %.2f s", runtime.seconds(), seconds);
            telemetry.update();
        }

        stopDrive();
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
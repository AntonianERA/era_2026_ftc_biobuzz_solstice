package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Wheel test: hold a button on gamepad1 to spin one wheel at 10% power.
//   X = front left   A = back left
//   Y = front right  B = back right
// Every wheel should spin FORWARD (the direction that drives the robot forward).
@TeleOp(name = "Wheel Test")
public class TestOpMode_WheelTest extends LinearOpMode {

    private static final double TEST_POWER = 0.1;

    @Override
    public void runOpMode() {

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Same directions as RobotControl, so this tests what the real opmode does.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("X = FL, A = BL, Y = FR, B = BR");
        telemetry.addLine("Each wheel should spin forward");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            frontLeft.setPower(gamepad1.x ? TEST_POWER : 0);
            backLeft.setPower(gamepad1.a ? TEST_POWER : 0);
            frontRight.setPower(gamepad1.y ? TEST_POWER : 0);
            backRight.setPower(gamepad1.b ? TEST_POWER : 0);

            telemetry.addLine("X = FL, A = BL, Y = FR, B = BR");
            telemetry.addData("Front Left  (X)", gamepad1.x ? "ON" : "off");
            telemetry.addData("Back Left   (A)", gamepad1.a ? "ON" : "off");
            telemetry.addData("Front Right (Y)", gamepad1.y ? "ON" : "off");
            telemetry.addData("Back Right  (B)", gamepad1.b ? "ON" : "off");
            telemetry.update();
        }
    }
}
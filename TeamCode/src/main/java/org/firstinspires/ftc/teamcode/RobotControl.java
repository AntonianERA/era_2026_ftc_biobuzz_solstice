package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;

/*
 * ROBOT CONTROL
 *
 * GAMEPAD 1 - DRIVE (field-centric)
 *   Left stick      drive / strafe
 *   Right stick X   turn
 *   Right trigger   slow mode (hold)
 *   Options         re-zero heading (robot facing away from you)
 *
 * GAMEPAD 2 - MECHANISMS
 *   X / Y           both intakes in / out
 *   A / B           intake2 only in / out
 *   DPAD UP / DOWN  intake only in / out
 *   Right bumper    shooter forward
 *   DPAD LEFT       shooter backward
 *   Left bumper     kicker
 */
@TeleOp(name = "Robot Control")
public class RobotControl extends LinearOpMode {

    private FieldCentricDrive drive;   // driveTrain/FieldCentricDrive
    private Intake intake;             // mechanisms/Intake
    private Outtake outtake;           // mechanisms/Outtake

    private static final double SLOW_MODE_SPEED = 0.35;   // right trigger slow mode

    @Override
    public void runOpMode() {

        // ---- INIT ----
        // ROBOT MUST BE STILL here (Pinpoint IMU calibrates).
        drive   = new FieldCentricDrive(hardwareMap);
        intake  = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // ---- LOOP ----
        while (opModeIsActive()) {

            // ================= GAMEPAD 1: DRIVE =================
            double forward = -gamepad1.left_stick_y;   // stick forward is negative
            double strafe  =  gamepad1.left_stick_x;
            double turn    =  gamepad1.right_stick_x;
            double speed   = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SPEED : 1.0;

            drive.drive(forward, strafe, turn, speed);

            if (gamepad1.options) {
                drive.resetHeading();
            }

            // ================= GAMEPAD 2: INTAKE =================
            double intakePower  = 0.0;
            double intake2Power = 0.0;

            if (gamepad2.x) {                 // both in
                intakePower  = Intake.IN_POWER;
                intake2Power = Intake.IN_POWER;
            } else if (gamepad2.y) {          // both out
                intakePower  = Intake.OUT_POWER;
                intake2Power = Intake.OUT_POWER;
            }

            if (gamepad2.a) {                 // intake2 only in
                intake2Power = Intake.IN_POWER;
            } else if (gamepad2.b) {          // intake2 only out
                intake2Power = Intake.OUT_POWER;
            }

            if (gamepad2.dpad_up) {           // intake only in
                intakePower = Intake.IN_POWER;
            } else if (gamepad2.dpad_down) {  // intake only out
                intakePower = Intake.OUT_POWER;
            }

            intake.setPower(intakePower, intake2Power);

            // ================= GAMEPAD 2: OUTTAKE =================
            if (gamepad2.right_bumper) {
                outtake.shoot();
            } else if (gamepad2.dpad_left) {
                outtake.reverse();
            } else {
                outtake.stopShooter();
            }

            if (gamepad2.left_bumper) {
                outtake.kick();
            } else {
                outtake.resetKicker();
            }

            // ================= TELEMETRY =================

            drive.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            outtake.addTelemetry(telemetry);
            telemetry.update();



//            double max;
//
//            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
//            double lateral =  -gamepad1.left_stick_x;
//            double yaw     =  gamepad1.right_stick_x;
//
//            // Combine the joystick requests for each axis-motion to determine each wheel's power.
//            // Set up a variable for each drive wheel to save the power level for telemetry.
//            double frontLeftPower  = axial + lateral + yaw;
//            double frontRightPower = axial - lateral - yaw;
//            double backLeftPower   = axial - lateral + yaw;
//            double backRightPower  = axial + lateral - yaw;
//
//            // Normalize the values so no wheel power exceeds 100%
//            // This ensures that the robot maintains the desired motion.
//            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//            max = Math.max(max, Math.abs(backLeftPower));
//            max = Math.max(max, Math.abs(backRightPower));
//
//            if (max > 1.0) {
//                frontLeftPower  /= max;
//                frontRightPower /= max;
//                backLeftPower   /= max;
//                backRightPower  /= max;
//            }
//
//            // This is test code:
//            //
//            // Uncomment the following code to test your motor directions.
//            // Each button should make the corresponding motor run FORWARD.
//            //   1) First get all the motors to take to correct positions on the robot
//            //      by adjusting your Robot Configuration if necessary.
//            //   2) Then make sure they run in the correct direction by modifying the
//            //      the setDirection() calls above.
//            // Once the correct motors move in the correct direction re-comment this code.
//
//            /*
//            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
//            */
//
//            // Send calculated power to wheels
//            frontLeftDrive.setPower(frontLeftPower);
//            frontRightDrive.setPower(frontRightPower);
//            backLeftDrive.setPower(backLeftPower);
//            backRightDrive.setPower(backRightPower);
//
//            // Show the elapsed game time and wheel power.
//
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
//            telemetry.addData("Velocity ", outake.getVelocity());
//            telemetry.addData("voltage ", outake.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Power ", outake.getPower());
//            telemetry.update();


        }


        // ---- STOP ----

    }
}
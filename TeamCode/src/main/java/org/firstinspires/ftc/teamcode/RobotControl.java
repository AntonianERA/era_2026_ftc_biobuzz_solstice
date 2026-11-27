package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Flower;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.mechanisms.ShootSequence;

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
 *   Right trigger   SHOOT 2 (spin up, kick, feed, kick, stop)
 *   Left trigger    SHOOT 1 (spin up, kick, stop)
 *                   (any intake/outtake button above cancels a shot)
 *   DPAD RIGHT      flower up    (placeholder)
 *   Right stick btn flower down  (placeholder)
 */
@TeleOp(name = "Robot Control")
public class RobotControl extends LinearOpMode {

    private FieldCentricDrive drive;   // driveTrain/FieldCentricDrive
    private Intake intake;             // mechanisms/Intake
    private Outtake outtake;           // mechanisms/Outtake
    private Flower flower;             // mechanisms/Flower
    private ShootSequence shooter;     // mechanisms/ShootSequence

    private static final double SLOW_MODE_SPEED = 0.35;   // right trigger slow mode

    @Override
    public void runOpMode() {

        // ---- INIT ----
        // ROBOT MUST BE STILL here (Pinpoint IMU calibrates).
        drive   = new FieldCentricDrive(hardwareMap);
        intake  = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        flower  = new Flower(hardwareMap);
        shooter = new ShootSequence(outtake, intake);

        boolean lastRightTrigger = false;
        boolean lastLeftTrigger  = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        flower.down();   // start in the down position

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

            // ================= GAMEPAD 2: SHOOT SEQUENCES =================
            boolean rightTrigger = gamepad2.right_trigger > 0.5;
            boolean leftTrigger  = gamepad2.left_trigger > 0.5;

            if (rightTrigger && !lastRightTrigger && !shooter.isRunning()) {
                shooter.shootDouble();          // spin up, kick, feed, kick, stop
            }
            if (leftTrigger && !lastLeftTrigger && !shooter.isRunning()) {
                shooter.shootOnce();            // spin up, kick, stop
            }
            lastRightTrigger = rightTrigger;
            lastLeftTrigger  = leftTrigger;

            // Any manual intake/outtake button cancels a running sequence
            boolean manualMechanism = gamepad2.x || gamepad2.y || gamepad2.a || gamepad2.b
                    || gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left
                    || gamepad2.right_bumper || gamepad2.left_bumper;
            if (manualMechanism && shooter.isRunning()) {
                shooter.cancel();
            }

            if (shooter.isRunning()) {
                shooter.update();               // sequence controls intake + outtake
            } else {
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

            }

            // ================= GAMEPAD 2: FLOWER =================
            // TODO: placeholder buttons - change once controls are decided
            if (gamepad2.dpad_right) {
                flower.up();
            } else if (gamepad2.right_stick_button) {
                flower.down();
            }

            // ================= TELEMETRY =================

            drive.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            outtake.addTelemetry(telemetry);
            flower.addTelemetry(telemetry);
            shooter.addTelemetry(telemetry);
            // DEBUG: raw gamepad2 inputs (remove once the shooter works)
            telemetry.addData("GP2 RT / LT", "%.2f / %.2f", gamepad2.right_trigger, gamepad2.left_trigger);
            telemetry.addData("GP2 RB / LB", "%b / %b", gamepad2.right_bumper, gamepad2.left_bumper);
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
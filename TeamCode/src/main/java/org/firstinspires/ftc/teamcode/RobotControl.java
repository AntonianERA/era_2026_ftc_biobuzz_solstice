package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.driveTrain.driveTrain;
import org.firstinspires.ftc.teamcode.driveTrain.geocentricDrive;

// Field-centric drive. Options = reset heading
@TeleOp(name = "Robot Control")
public class RobotControl extends LinearOpMode {

    private driveTrain driveTrain;
    private DcMotor intake;
    private DcMotor outake1;
    private DcMotor outake2;

    @Override
    public void runOpMode() {
        // ---- INIT ----
        driveTrain = new driveTrain(hardwareMap);

        telemetry.addLine("Initialized - point robot away from you, then press Start");
        telemetry.update();

        waitForStart();
        driveTrain.resetHeading();

        // ---- LOOP ----
        while (opModeIsActive()) {
            if (gamepad1.options) driveTrain.resetHeading();

            geocentricDrive.drive(
                    driveTrain,
                    gamepad1.left_stick_x,   // strafe
                    -gamepad1.left_stick_y,  // forward (stick Y is inverted)
                    gamepad1.right_stick_x); // turn

            telemetry.addData("Heading (deg)", "%.1f", driveTrain.getHeadingDegrees());
            telemetry.update();

            if (gamepad2.right_bumper){
                outake1.setPower(1);
            }else{
                outake1.setPower(0);
            }

            if(gamepad2.left_bumper){
                outake2.setPower(-1);
            }else{
                outake2.setPower(0);
            }

            if(gamepad2.x){
                intake.setPower(1);

            }else if (gamepad2.y){
                intake.setPower(-1);
            }else{
                intake.setPower(0);
            }
        }

        // ---- STOP ----
        driveTrain.stop();
    }
}
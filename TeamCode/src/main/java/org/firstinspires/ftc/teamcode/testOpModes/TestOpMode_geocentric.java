package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.driveTrain;
import org.firstinspires.ftc.teamcode.driveTrain.geocentricDrive;

// Drive-only test for field-centric. Options = reset heading
@TeleOp(name = "TEST Geocentric")
public class TestOpMode_geocentric extends LinearOpMode {

    @Override
    public void runOpMode() {
        driveTrain driveTrain = new driveTrain(hardwareMap);

        telemetry.addLine("Ready - point robot away from you, then press Start");
        telemetry.update();

        waitForStart();
        driveTrain.resetHeading();

        while (opModeIsActive()) {
            if (gamepad1.options) driveTrain.resetHeading();

            geocentricDrive.drive(
                    driveTrain,
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x);

            telemetry.addData("Heading (deg)", "%.1f", driveTrain.getHeadingDegrees());
            telemetry.addData("FL / FR", "%.2f / %.2f",
                    driveTrain.fl.getPower(), driveTrain.fr.getPower());
            telemetry.addData("BL / BR", "%.2f / %.2f",
                    driveTrain.bl.getPower(), driveTrain.br.getPower());
            telemetry.update();
        }

        driveTrain.stop();
    }
}
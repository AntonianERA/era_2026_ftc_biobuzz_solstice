package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Finds the "zero" (rest) position for the "flower" servo.
 * Step the servo until the mechanism sits where you want it, then copy the
 * position shown on the Driver Station into your code.
 *
 * Controls (gamepad 1):
 *   DPAD UP / DOWN       +0.01 / -0.01  (fine)
 *   RIGHT / LEFT BUMPER  +0.05 / -0.05  (coarse)
 *   Y                    jump back to the middle (0.50)
 *
 * The servo starts at the middle (0.50) when you press play, so it doesn't
 * slam into either end of the mechanism.
 */
@TeleOp(name = "Flower Servo Test", group = "Test")
public class TestOpMode_flower extends LinearOpMode {

    private static final double START_POSITION = 0.50;
    private static final double FINE_STEP = 0.01;
    private static final double COARSE_STEP = 0.05;

    @Override
    public void runOpMode() {
        Servo flower = hardwareMap.get(Servo.class, "flower");

        double position = START_POSITION;
        boolean lastUp = false, lastDown = false, lastRB = false, lastLB = false, lastY = false;

        telemetry.addData("Status", "Initialized - press start");
        telemetry.addLine("Servo will move to 0.50 when you press start.");
        telemetry.update();

        waitForStart();

        flower.setPosition(position);

        while (opModeIsActive()) {
            // Edge detection: one step per press, not every loop
            if (gamepad1.dpad_up && !lastUp)         position += FINE_STEP;
            if (gamepad1.dpad_down && !lastDown)     position -= FINE_STEP;
            if (gamepad1.right_bumper && !lastRB)    position += COARSE_STEP;
            if (gamepad1.left_bumper && !lastLB)     position -= COARSE_STEP;
            if (gamepad1.y && !lastY)                position = START_POSITION;

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastRB = gamepad1.right_bumper;
            lastLB = gamepad1.left_bumper;
            lastY = gamepad1.y;

            // Keep within valid servo range and clean up float drift
            position = Range.clip(position, 0.0, 1.0);
            position = Math.round(position * 100.0) / 100.0;

            flower.setPosition(position);

            telemetry.addData("Flower Position", "%.2f", position);
            telemetry.addLine();
            telemetry.addData("Fine", "DPAD UP / DOWN  (+/- 0.01)");
            telemetry.addData("Coarse", "RB / LB  (+/- 0.05)");
            telemetry.addData("Reset", "Y  (back to 0.50)");
            telemetry.addLine();
            telemetry.addLine("If the servo buzzes or strains, back off -");
            telemetry.addLine("it's pushing against the mechanism.");
            telemetry.update();
        }
    }
}

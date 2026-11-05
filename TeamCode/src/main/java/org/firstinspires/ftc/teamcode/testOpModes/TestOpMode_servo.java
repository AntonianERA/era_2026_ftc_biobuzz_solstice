package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Test")
public class TestOpMode_servo extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Change "servo" to match the name in your robot configuration
        Servo servo = hardwareMap.get(Servo.class, "servo");

        double position = 0.0;
        boolean lastA = false;
        boolean lastB = false;

        telemetry.addData("Status", "Initialized - press start");
        telemetry.update();

        waitForStart();

        servo.setPosition(position);

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            // Edge detection: only step once per press, not every loop
            if (a && !lastA) {
                position += 0.1;
            }
            if (b && !lastB) {
                position += 0.05;
            }

            // Keep within valid servo range and clean up float drift
            position = Range.clip(position, 0.0, 1.0);
            position = Math.round(position * 100.0) / 100.0;

            servo.setPosition(position);

            lastA = a;
            lastB = b;

            telemetry.addData("Servo Position", "%.2f", position);
            telemetry.addData("Controls", "A: +0.10 | B: +0.05");
            telemetry.update();
        }
    }
}
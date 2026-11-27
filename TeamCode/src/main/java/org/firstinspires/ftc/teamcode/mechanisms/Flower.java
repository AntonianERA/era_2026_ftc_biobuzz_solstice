package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flower system: servo "flower".
 * Hardware + actions only - the buttons that call these live in RobotControl.
 * Positions found with TestOpMode_flower.
 */
public class Flower {

    public static final double UP_POS   = 0.4;
    public static final double DOWN_POS = 0.0;

    private final Servo flower;
    private boolean isUp = false;

    public Flower(HardwareMap hardwareMap) {
        flower = hardwareMap.get(Servo.class, "flower");
    }

    public void up() {
        flower.setPosition(UP_POS);
        isUp = true;
    }

    public void down() {
        flower.setPosition(DOWN_POS);
        isUp = false;
    }

    /** Switches between up and down. */
    public void toggle() {
        if (isUp) down();
        else      up();
    }

    public boolean isUp() { return isUp; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower", isUp ? "UP" : "DOWN");
    }
}

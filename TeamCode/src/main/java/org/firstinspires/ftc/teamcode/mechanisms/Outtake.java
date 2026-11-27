package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Outtake system: shooter motor "outake" (velocity control) and the "kicker" servo.
 * Hardware + actions only - the buttons that call these live in RobotControl.
 */
public class Outtake {

    public static final double SHOOT_RPM     = 2150;
    public static final double REVERSE_RPM   = 1900;
    public static final double TICKS_PER_REV = 28;   // motor encoder ticks per revolution

    // ---- Shooter velocity PIDF (stops the RPM overshooting the target) ----
    // F: power needed to hold a speed. Start = 32767 / max ticks per second.
    //    5203 6000 RPM bare motor: 6000 * 28 / 60 = 2800 ticks/s -> F = 11.7
    // P: start at 0.1 * F.  I: start at 0.1 * P.  D: leave at 0.
    // Still overshoots -> lower P and I.  Slow to reach speed / sits below -> raise P (or F).
    public static final double VEL_F = 16;
    public static final double VEL_P = 16;
    public static final double VEL_I = 0.117;
    public static final double VEL_D = 0.0;

    public static final double KICK_POS = 0.0;
    public static final double REST_POS = 0.15;

    private final DcMotorEx outake;
    private final Servo kicker;

    private double targetRpm = 0.0;

    public Outtake(HardwareMap hardwareMap) {
        outake = hardwareMap.get(DcMotorEx.class, "outake");
        outake.setDirection(DcMotorSimple.Direction.REVERSE);
        outake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // same setup as the RPM Tuner
        outake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);      // needed for setVelocity
        outake.setVelocityPIDFCoefficients(VEL_P, VEL_I, VEL_D, VEL_F);
        outake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker = hardwareMap.get(Servo.class, "kicker");
    }

    // ----- Shooter -----
    public void shoot()        { setRpm(SHOOT_RPM); }
    public void reverse()      { setRpm(-REVERSE_RPM); }
    public void stopShooter()  { setRpm(0); }

    /** Shooter speed in RPM (negative = backward). */
    public void setRpm(double rpm) {
        targetRpm = rpm;
        outake.setVelocity(rpm * TICKS_PER_REV / 60.0); // RPM -> ticks per second
    }

    public double getRpm() {
        return outake.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    /** True when the shooter is within tolerance of its target speed. */
    public boolean atSpeed(double toleranceRpm) {
        return targetRpm != 0 && Math.abs(getRpm() - targetRpm) <= toleranceRpm;
    }

    // ----- Kicker -----
    public void kick()        { kicker.setPosition(KICK_POS); }
    public void resetKicker() { kicker.setPosition(REST_POS); }

    public void stop() {
        stopShooter();
        resetKicker();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM (target / actual)", "%.0f / %.0f", targetRpm, getRpm());
        telemetry.addData("Shooter current (mA)", "%.0f", outake.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Shooter power / mode", "%.2f / %s", outake.getPower(), outake.getMode());
    }
}

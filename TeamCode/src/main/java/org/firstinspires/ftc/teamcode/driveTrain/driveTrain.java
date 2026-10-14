package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Holds the mecanum drivetrain hardware (4 motors + IMU) and basic helpers.
 * Create one in your OpMode with: DriveTrain drive = new DriveTrain(hardwareMap);
 */
public class driveTrain {

    public final DcMotor fl, bl, fr, br;
    public final IMU imu;

    public driveTrain(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("frontLeft");
        bl = hardwareMap.dcMotor.get("backLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        br = hardwareMap.dcMotor.get("backRight");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    }

    /** Sets all four wheel powers at once. */
    public void setPowers(double flPower, double blPower, double frPower, double brPower) {
        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /** Robot heading in radians. */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Robot heading in degrees (handy for telemetry). */
    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}
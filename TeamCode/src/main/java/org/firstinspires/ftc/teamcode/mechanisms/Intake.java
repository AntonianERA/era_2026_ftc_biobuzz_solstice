package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake system: motors "intake" and "intake2".
 * Hardware + actions only - the buttons that call these live in RobotControl.
 */
public class Intake {

    public static final double IN_POWER  = 1.0;
    public static final double OUT_POWER = -1.0;

    private final DcMotor intake;
    private final DcMotor intake2;

    public Intake(HardwareMap hardwareMap) {
        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
    }

    /** Set each motor directly (-1 to 1). */
    public void setPower(double intakePower, double intake2Power) {
        intake.setPower(intakePower);
        intake2.setPower(intake2Power);
    }

    public void setIntakePower(double power)  { intake.setPower(power); }
    public void setIntake2Power(double power) { intake2.setPower(power); }

    /** Both motors in / out / off. */
    public void runIn()  { setPower(IN_POWER, IN_POWER); }
    public void runOut() { setPower(OUT_POWER, OUT_POWER); }
    public void stop()   { setPower(0, 0); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake / Intake2", "%.2f / %.2f", intake.getPower(), intake2.getPower());
    }
}

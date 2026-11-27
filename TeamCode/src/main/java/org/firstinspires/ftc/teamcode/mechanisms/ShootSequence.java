package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Automatic shooting sequences that use the Outtake and Intake together.
 *
 *   Double shot: spin up -> kick -> run intake to load ball 2 -> kick -> shooter off
 *   Single shot: spin up -> kick -> shooter off
 *
 * Non-blocking: start one with shootDouble() / shootOnce(), then call update()
 * every loop. The robot keeps driving while it runs. cancel() stops it.
 */
public class ShootSequence {

    // ---- Timing (seconds) - tune these on the robot ----
    public static final double SPIN_UP_TIME = 3.0;   // shooter spin-up before the first kick
    public static final double KICK_TIME    = 0.3;   // how long the kicker stays out (placeholder - tune)
    public static final double FEED_TIME    = 3.0;   // intake run time to load the second ball

    private enum Step { IDLE, SPIN_UP, KICK_1, FEED, KICK_2 }

    private final Outtake outtake;
    private final Intake intake;
    private final ElapsedTime timer = new ElapsedTime();

    private Step step = Step.IDLE;
    private boolean doubleShot = false;

    public ShootSequence(Outtake outtake, Intake intake) {
        this.outtake = outtake;
        this.intake = intake;
    }

    /** Two balls: spin up, kick, feed with intake, kick again, stop. */
    public void shootDouble() {
        start(true);
    }

    /** One ball: spin up, kick, stop. */
    public void shootOnce() {
        start(false);
    }

    private void start(boolean twoBalls) {
        doubleShot = twoBalls;
        intake.stop();
        outtake.resetKicker();
        outtake.shoot();
        goTo(Step.SPIN_UP);
    }

    /** Call once per loop. Does nothing when no sequence is running. */
    public void update() {
        switch (step) {
            case IDLE:
                break;

            case SPIN_UP:
                outtake.shoot();
                if (timer.seconds() >= SPIN_UP_TIME) {
                    outtake.kick();
                    goTo(Step.KICK_1);
                }
                break;

            case KICK_1:
                if (timer.seconds() >= KICK_TIME) {
                    outtake.resetKicker();
                    if (doubleShot) {
                        intake.runIn();
                        goTo(Step.FEED);
                    } else {
                        finish();
                    }
                }
                break;

            case FEED:
                if (timer.seconds() >= FEED_TIME) {
                    intake.stop();
                    outtake.kick();
                    goTo(Step.KICK_2);
                }
                break;

            case KICK_2:
                if (timer.seconds() >= KICK_TIME) {
                    finish();
                }
                break;
        }
    }

    /** Stops everything and ends the sequence. */
    public void cancel() {
        finish();
    }

    public boolean isRunning() {
        return step != Step.IDLE;
    }

    private void finish() {
        outtake.resetKicker();
        outtake.stopShooter();
        intake.stop();
        step = Step.IDLE;
    }

    private void goTo(Step next) {
        step = next;
        timer.reset();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shoot sequence", step == Step.IDLE ? "idle"
                : (doubleShot ? "DOUBLE: " : "SINGLE: ") + step);
    }
}

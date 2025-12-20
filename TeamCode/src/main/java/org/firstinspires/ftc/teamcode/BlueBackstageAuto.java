package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blue backstage auto")
public class BlueBackstageAuto extends LinearOpMode {

    private final IntakeLiftCamera ILC = new IntakeLiftCamera();
    private final Drivetrain DT = new Drivetrain(this);

    // ---- Tune these for your robot ----
    private static final double DRIVE_SPEED = 0.3;
    private static final double DRIVE_DISTANCE_IN = 6.5;

    // Use the same power as your ILC.longShoot()
    private static final double SHOOTER_POWER = 0.875;

    // "few seconds" before pushing lever to feed the ball
    private static final long SHOOTER_REV_TIME_MS = 4500;

    // How long to hold lever up to push one ball into shooter
    private static final long LEVER_PUSH_MS = 600;

    // Time for the ball to clear + shooter to recover
    private static final long BETWEEN_SHOTS_MS = 4500;

    // Sorter settings
    private static final double TICKS_PER_REV = 537.6;          // matches your motors
    private static final int SORTER_STEPS_PER_REV = 6;          // 1/6 per click
    private static final int STEPS_PER_BALL = 2;                // "click twice to a third" (2/6 = 1/3)
    private static final long SORTER_TIMEOUT_MS = 900;
    // -----------------------------------

    private volatile boolean keepPitchRunning = false;
    private Thread keepPitchThread;

    @Override
    public void runOpMode() {
        ILC.initILC(hardwareMap);
        DT.initDrivetrain(hardwareMap);
        DT.initGyro(hardwareMap);

        // Safe init states
        ILC.leverDown();
        ILC.shootingOff();

        telemetry.addLine("Ready: ThreeBallAuto_Preloaded");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Keep pitch servo commanded throughout the entire auto
        keepPitchRunning = true;
        keepPitchThread = new Thread(() -> {
            while (keepPitchRunning && !Thread.currentThread().isInterrupted()) {
                ILC.pitch();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        keepPitchThread.start();

        // Move up 6 inches
        DT.drive(DRIVE_SPEED, DRIVE_DISTANCE_IN);

        // Rev shooter, then fire 3 balls (already inside robot)
        setShooterPower(SHOOTER_POWER);
        waitMs(SHOOTER_REV_TIME_MS);
        waitBetweenBalls();

        fireOneBall();
        advanceSorterOneThird();
        waitBetweenBalls();

        fireOneBall();
        advanceSorterOneThird();
        waitMs(3000);

        fireOneBall();
        waitMs(1000);
        // Stop everything
        ILC.leverDown();
        ILC.shootingOff();

        DT.drive(0.4, 12);

        // Stop pitch-maintain thread
        keepPitchRunning = false;
        if (keepPitchThread != null) keepPitchThread.interrupt();
    }

    private void fireOneBall() {
        if (!opModeIsActive() || isStopRequested()) return;

        // Make sure shooter is still spinning
        setShooterPower(SHOOTER_POWER);

        // Push ball into shooter
        ILC.leverUp();
        waitMs(LEVER_PUSH_MS);
        ILC.leverDown();
    }

    private void waitBetweenBalls() {
        // Clear + recover
        waitMs(BETWEEN_SHOTS_MS);
    }

    private void advanceSorterOneThird() {
        rotateSorterSteps(STEPS_PER_BALL);
    }

    private void rotateSorterSteps(int steps) {
        if (!opModeIsActive() || isStopRequested()) return;
        if (ILC.sorterMotor == null) return; // safety

        int ticksPerStep = (int) (TICKS_PER_REV / SORTER_STEPS_PER_REV);
        int start = ILC.sorterMotor.getCurrentPosition();
        int target = start + ticksPerStep * steps;

        ILC.sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ILC.sorterMotor.setTargetPosition(target);
        ILC.sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ILC.sorterMotor.setPower(0.75);

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && !isStopRequested()
                && ILC.sorterMotor.isBusy()
                && t.milliseconds() < SORTER_TIMEOUT_MS) {
            idle();
        }

        ILC.sorterMotor.setPower(0);
        ILC.sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setShooterPower(double power) {
        // Using your existing ILC methods keeps behavior consistent
        if (Math.abs(power - 0.875) < 1e-6) {
            ILC.longShoot();
        } else if (Math.abs(power - 0.795) < 1e-6) {
            ILC.shortShoot();
        } else {
            // direct access (same package) if you want arbitrary power:
            if (ILC.shootingMotor != null) ILC.shootingMotor.setPower(power);
        }
    }

    private void waitMs(long ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && !isStopRequested() && timer.milliseconds() < ms) {
            idle();
        }
    }
}

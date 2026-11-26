package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class TurretTracking {
    private static final double TICKS_PER_DEGREE = 384.5 * 2.75 / 360.0;
    private static final double LIMIT_DEGREES = 180.0;
    private static final int LIMIT_TICKS = (int) (LIMIT_DEGREES * TICKS_PER_DEGREE);
    private static final int TARGET_TOLERANCE_TICKS = 2;
    private static final double START_TRACKING_DEGREES = 2.0;
    private static final double STOP_TRACKING_DEGREES = 1.0;
    private static final double SLOWDOWN_DEGREES = 25.0;
    private static final double INITIAL_SPEED_FRACTION = 0.15;
    private static final double SPEED_RAMP_PER_SECOND = 3.0;
    private static final double REWIND_SPEED = 90.0 * TICKS_PER_DEGREE;
    private static final double MAX_SPEED_FRACTION = 0.75;
    private static final long REWIND_TIMEOUT_NANOS = 6_000_000_000L;

    // Positive encoder movement must turn the turret right. Verify this on the robot.
    private static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    private final DcMotorEx motor;
    private final int forwardTicks;
    private final int fullSpeedTicksPerSecond;
    private double lastFrameTimestamp = Double.NaN;
    private double trackingSpeedFraction;
    private long lastTrackingUpdateNanos;
    private int trackingDirection;
    private long rewindStartedNanos;
    private int targetTicks;
    private boolean rewinding;
    private boolean moving;

    TurretTracking(DcMotorEx motor) {
        this.motor = motor;
        motor.setDirection(MOTOR_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forwardTicks = motor.getCurrentPosition();
        fullSpeedTicksPerSecond = Math.max(1, motor.getMotorType().getAchieveableMaxTicksPerSecondRounded());
        motor.setTargetPositionTolerance(TARGET_TOLERANCE_TICKS);
        motor.setTargetPosition(forwardTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
        targetTicks = forwardTicks;
    }

    void update(double tx, double frameTimestamp) {
        int currentTicks = motor.getCurrentPosition();
        if (rewinding) {
            // The tag will leave view while crossing forward. Finish the last camera target first.
            if (Math.abs(currentTicks - targetTicks) <= TARGET_TOLERANCE_TICKS
                    || System.nanoTime() - rewindStartedNanos >= REWIND_TIMEOUT_NANOS) {
                stop();
                lastFrameTimestamp = frameTimestamp;
            }
            return;
        }

        if (!Double.isFinite(tx)) {
            if (moving) {
                stop();
            }
            return;
        }
        if (frameTimestamp == lastFrameTimestamp) {
            return;
        }
        lastFrameTimestamp = frameTimestamp;

        double deadband = moving ? STOP_TRACKING_DEGREES : START_TRACKING_DEGREES;
        if (Math.abs(tx) <= deadband) {
            if (moving) {
                stop();
            }
            return;
        }

        // The camera rotates with the turret. Its reported tx is positive to the right.
        double heading = (currentTicks - forwardTicks) / TICKS_PER_DEGREE + tx;
        if (heading > LIMIT_DEGREES) {
            heading -= 360.0;
            rewinding = true;
        } else if (heading < -LIMIT_DEGREES) {
            heading += 360.0;
            rewinding = true;
        }

        int relativeTicks = (int) Math.round(heading * TICKS_PER_DEGREE);
        targetTicks = forwardTicks + Math.max(-LIMIT_TICKS, Math.min(LIMIT_TICKS, relativeTicks));
        motor.setTargetPosition(targetTicks);
        // REV RUN_TO_POSITION uses positive power for either direction and scales it to speed.
        if (rewinding) {
            motor.setPower(Math.min(MAX_SPEED_FRACTION, REWIND_SPEED / fullSpeedTicksPerSecond));
            rewindStartedNanos = System.nanoTime();
        } else {
            updateTrackingSpeed(currentTicks);
        }
        moving = true;
    }

    private void updateTrackingSpeed(int currentTicks) {
        int direction = Integer.compare(targetTicks, currentTicks);
        double degreesRemaining = Math.abs(targetTicks - currentTicks) / TICKS_PER_DEGREE;
        double requestedSpeed = MAX_SPEED_FRACTION * Math.min(1.0, degreesRemaining / SLOWDOWN_DEGREES);

        long now = System.nanoTime();
        double rampLimit = INITIAL_SPEED_FRACTION;
        if (moving && direction == trackingDirection) {
            // Cap the time step so a delayed camera frame cannot jump straight to full speed.
            double elapsedSeconds = Math.min(0.05, (now - lastTrackingUpdateNanos) * 1e-9);
            rampLimit = trackingSpeedFraction + SPEED_RAMP_PER_SECOND * elapsedSeconds;
        }
        trackingSpeedFraction = Math.min(requestedSpeed, rampLimit);
        motor.setPower(trackingSpeedFraction);
        trackingDirection = direction;
        lastTrackingUpdateNanos = now;
    }

    void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", "%.1f deg", (motor.getCurrentPosition() - forwardTicks)
                / TICKS_PER_DEGREE);
        telemetry.addData("Turret", rewinding ? "Rewinding" : moving ? "Tracking" : "Stopped");
    }

    void stop() {
        motor.setPower(0.0);
        moving = false;
        rewinding = false;
    }
}

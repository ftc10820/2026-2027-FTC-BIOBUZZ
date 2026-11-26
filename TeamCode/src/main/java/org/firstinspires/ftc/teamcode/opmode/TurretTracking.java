package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

final class TurretTracking {
    private static final double TICKS_PER_DEGREE = 384.5 * 2.75 / 360.0;
    private static final double LIMIT_DEGREES = 180.0;
    private static final int LIMIT_TICKS = (int) (LIMIT_DEGREES * TICKS_PER_DEGREE);
    private static final int TARGET_TOLERANCE_TICKS = 2;
    private static final double STOPPED_SPEED_TICKS_PER_SECOND = 5.0;
    private static final double START_TRACKING_DEGREES = 2.0;
    private static final double STOP_TRACKING_DEGREES = 1.0;
    private static final double SLOWDOWN_DEGREES = 25.0;
    private static final double LIMIT_SLOWDOWN_DEGREES = 45.0;
    private static final double REWIND_EDGE_DEGREES = 6.0;
    private static final double REACQUIRE_CLEARANCE_DEGREES = 30.0;
    private static final double INITIAL_SPEED_FRACTION = 0.15;
    private static final double SPEED_RAMP_PER_SECOND = 3.0;
    private static final double MAX_SPEED_FRACTION = 1.0;
    private static final long REWIND_TIMEOUT_NANOS = 6_000_000_000L;

    // Positive encoder movement must turn the turret right. Verify this on the robot.
    private static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    private final DcMotorEx motor;
    private final int forwardTicks;
    private double lastFrameTimestamp = Double.NaN;
    private double speedFraction;
    private long lastSpeedUpdateNanos;
    private int lastDirection;
    private long rewindStartedNanos;
    private int rewindStartTicks;
    // Turret positions in ticks relative to the encoder reading captured at INIT.
    private int targetTicks;
    private boolean rewinding;
    private boolean moving;
    private boolean rewindOnTagLoss;
    private boolean brakingAtLimit;

    TurretTracking(DcMotorEx motor) {
        this.motor = motor;
        motor.setDirection(MOTOR_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forwardTicks = motor.getCurrentPosition();
        motor.setTargetPositionTolerance(TARGET_TOLERANCE_TICKS);
        motor.setTargetPosition(forwardTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.0);
    }

    void update(double tx, double frameTimestamp) {
        int currentTicks = motor.getCurrentPosition() - forwardTicks;
        if (lastDirection != 0 && lastDirection * currentTicks >= LIMIT_TICKS) {
            // Cut outward drive as soon as the encoder reaches either end of travel.
            brakeMotor();
            brakingAtLimit = true;
            return;
        }
        if (brakingAtLimit) {
            // Let the turret come to rest before tracking or rewind can command it inward.
            if (!(Math.abs(motor.getVelocity()) <= STOPPED_SPEED_TICKS_PER_SECOND)) {
                return;
            }
            brakingAtLimit = false;
        }

        double currentDegrees = currentTicks / TICKS_PER_DEGREE;
        long now = System.nanoTime();
        boolean tagSeen = Double.isFinite(tx) && Double.isFinite(frameTimestamp);
        boolean newFrame = tagSeen && frameTimestamp != lastFrameTimestamp;
        if (newFrame) {
            lastFrameTimestamp = frameTimestamp;
        }

        // The camera turns with the turret, and its reported tx is positive to the right.
        double heading = currentDegrees + tx;
        if (rewinding) {
            // Let the original view clear before using a new tag to interrupt the sweep.
            boolean clearedOriginalView = Math.abs(currentTicks - rewindStartTicks)
                    >= REACQUIRE_CLEARANCE_DEGREES * TICKS_PER_DEGREE;
            if (newFrame && clearedOriginalView && Math.abs(heading) <= LIMIT_DEGREES) {
                rewinding = false;
            } else {
                if (Math.abs(currentTicks - targetTicks) <= TARGET_TOLERANCE_TICKS
                        || now - rewindStartedNanos >= REWIND_TIMEOUT_NANOS) {
                    stop();
                } else {
                    updateMotorSpeed(currentTicks, now);
                }
                return;
            }
        }

        if (!tagSeen) {
            if (rewindOnTagLoss && Math.abs(currentDegrees) >= LIMIT_DEGREES - REWIND_EDGE_DEGREES) {
                startRewind(currentTicks, Integer.signum(currentTicks), now);
            } else if (Math.abs(currentTicks) > LIMIT_TICKS) {
                // Recover inward if the mechanism has coasted past the encoder limit.
                setTargetTicks(Integer.signum(currentTicks) * (LIMIT_TICKS - TARGET_TOLERANCE_TICKS));
            } else if (moving) {
                stop();
            }
        } else if (newFrame) {
            // Check travel before the deadband so an outward error at the limit can trigger rewind.
            if (Math.abs(heading) > LIMIT_DEGREES) {
                startRewind(currentTicks, heading > 0.0 ? 1 : -1, now);
            } else {
                double deadband = moving ? STOP_TRACKING_DEGREES : START_TRACKING_DEGREES;
                if (Math.abs(tx) <= deadband && Math.abs(currentTicks) <= LIMIT_TICKS) {
                    if (moving) {
                        stop();
                    }
                } else {
                    setTargetTicks((int) Math.round(heading * TICKS_PER_DEGREE));
                }
                // A valid aim at the edge can leave the camera's view between two frames.
                rewindOnTagLoss = Math.abs(heading) >= LIMIT_DEGREES - REWIND_EDGE_DEGREES;
            }
        }

        if (moving) {
            updateMotorSpeed(currentTicks, now);
        }
    }

    private void startRewind(int currentTicks, int limitSide, long now) {
        rewindOnTagLoss = false;
        rewinding = true;
        rewindStartTicks = currentTicks;
        rewindStartedNanos = now;
        setTargetTicks(-limitSide * LIMIT_TICKS);
    }

    private void setTargetTicks(int requestedTicks) {
        int boundedTicks = Math.max(-LIMIT_TICKS, Math.min(LIMIT_TICKS, requestedTicks));
        if (boundedTicks != targetTicks) {
            targetTicks = boundedTicks;
            motor.setTargetPosition(forwardTicks + targetTicks);
        }
        moving = true;
    }

    private void updateMotorSpeed(int currentTicks, long now) {
        int direction = Integer.compare(targetTicks, currentTicks);
        double targetDistance = Math.abs(targetTicks - currentTicks) / TICKS_PER_DEGREE;
        double limitDistance = (LIMIT_TICKS - direction * currentTicks) / TICKS_PER_DEGREE;
        double speedLimit = Math.min(limitDistance / LIMIT_SLOWDOWN_DEGREES,
                                     targetDistance / SLOWDOWN_DEGREES);
        double requestedSpeed = MAX_SPEED_FRACTION * Math.max(0.0, Math.min(1.0, speedLimit));
        double rampLimit = INITIAL_SPEED_FRACTION;
        if (direction == lastDirection) {
            // Limit the ramp step if the OpMode loop stalls.
            double elapsedSeconds = Math.min(0.05, (now - lastSpeedUpdateNanos) * 1e-9);
            rampLimit = speedFraction + SPEED_RAMP_PER_SECOND * elapsedSeconds;
        }
        // REV RUN_TO_POSITION uses positive power for both directions and scales it to speed.
        speedFraction = Math.min(requestedSpeed, rampLimit);
        motor.setPower(speedFraction);
        lastDirection = direction;
        lastSpeedUpdateNanos = now;
    }

    void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle", "%.1f deg", (motor.getCurrentPosition() - forwardTicks)
                / TICKS_PER_DEGREE);
        telemetry.addData("Turret", rewinding ? "Rewinding" : moving ? "Tracking" : "Stopped");
    }

    void stop() {
        brakeMotor();
        rewindOnTagLoss = false;
        brakingAtLimit = false;
    }

    private void brakeMotor() {
        motor.setPower(0.0);
        lastDirection = 0;
        moving = false;
        rewinding = false;
    }
}

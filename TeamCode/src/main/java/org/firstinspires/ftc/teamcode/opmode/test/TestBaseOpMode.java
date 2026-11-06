package org.firstinspires.ftc.teamcode.opmode.test;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

import org.firstinspires.ftc.teamcode.PIDConstants;

public abstract class TestBaseOpMode extends InitOpMode {
    // Constants
    public static double FRONT_GOAL_Y = -20;
    public static double BACK_GOAL_Y = 20;
    public static double RED_GOAL_X = 18;
    public static double BLUE_GOAL_X = -18;

    // State members
    protected double goalPositionY = FRONT_GOAL_Y;
    protected double goalPositionX = RED_GOAL_X;
    protected double targetVelocity = PIDConstants.SHOOTER_IDLE_VELOCITY;
    protected double targetTurretAngle = 0d;

    protected boolean isAimed() {
        return (Math.floor(1000*targetTurretAngle/getTurretRotation()) < 10);
    }

    protected boolean isAtSpeed() {
        return (Math.floor(1000*targetVelocity/shooterMotor.getVelocity()) <= 10);
    }

    protected boolean checkHiveGoal() {
        // TODO: Use vision to determine if goal is at front or back of field to update goalPositionY
        // TODO: look for AprilTag
        // TODO: If no AprilTag detected, look for hive structure
        // TODO: Return true if successful
        return true;
    }

    protected void detectGoal() {
        // Try up to 10 times
        for (int i=0;i<10;i++) {
            if (checkHiveGoal()) { return; }

            // Failed to set hive goal; sleep, move and try again
            // TODO: Move drive/camera
            sleep(10);
        }
    }

    protected double getTurretRotation() {
        // TODO: Find angle between current pose and goal
        // goalPositionX, goalPositionY
        // drive.localizer.getPose()
        // TODO: Translate the angle to turret servo position
        return targetTurretAngle;
    }

    protected double calculateRpmForDistance() {
        // TODO: Calculate distance between current pose and goal
        // goalPositionX, goalPositionY
        // drive.localizer.getPose()
        // TODO: Translate distance to shooter speed
        return targetVelocity;
    }

    protected void updateShooter() {
        // If there are balls loaded and we're currently aimed, set shooter speed
        if (!isEmpty() && isAimed()) {
            targetVelocity = PIDConstants.rpmToTicks(calculateRpmForDistance());
        }
        shooterMotor.setVelocity(targetVelocity);
    }
    protected void feedShooter() {
        intakeMotor.setPower(1);
        stopperServo.setPosition(STOPPER_OPEN_POSITION);
    }

    protected boolean isEmpty() {
        // TODO: Use sensor to detect balls
        return true;
    }

    protected void idleShooter() {
        targetVelocity = PIDConstants.SHOOTER_IDLE_VELOCITY;
        shooterMotor.setVelocity(targetVelocity);
        intakeMotor.setPower(0);
        stopperServo.setPosition(STOPPER_CLOSED_POSITION);
    }


}

package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PIDConstants;
import org.firstinspires.ftc.teamcode.opmode.test.TestBaseOpMode;

@Autonomous(name = "Test Auto Target", group="Test")
public class TestAutoTarget extends TestBaseOpMode {

    public void runOpMode() {
        initialize();
        // TODO: Initialize goalPositionX based on alliance color
        waitForStart();
/*
        // Do stuff then drive but not at the same time
        if (opModeIsActive()) {
            // Turn on shooter idle and detect the hive goal
            Actions.runBlocking(
                    new SequentialAction(
                    p -> { shooterMotor.setVelocity(PIDConstants.SHOOTER_IDLE_VELOCITY); return false; },
                    p -> { detectGoal(); return false; }
                    )
            );

            // Aim turret while turn on shooter at shooting speed
            // Feed the shooter until empty
            //The idle the shooter, reset the stopper, and turn off the intake
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    p -> { moveTurretByDegrees(getTurretRotation()); return isAimed(); },
                                    p -> { shooterMotor.setVelocity(targetVelocity = PIDConstants.rpmToTicks(calculateRpmForDistance())); return isAtSpeed(); }
                            ),
                            p -> { feedShooter(); return isEmpty(); },
                            p -> { idleShooter(); return false; }
                    )
            );

            // TODO: Drive, pick up new balls, re-target and shoot, drive, pick up more balls, re-target and shoot, park in loading
        }
*/
        // Do stuff and drive
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            // Check goal
                            p -> { detectGoal(); return false; },
                            new ParallelAction(
                                // Set shooter speed
                                p -> { updateShooter(); return isAtSpeed(); },
                                // Re-aim turret
                                p -> { moveTurretByDegrees(getTurretRotation()); return isAimed(); }
                            ),
                            // If there are balls loaded, feed the shooter
                            p -> { if (!isEmpty() && isAimed() && isAtSpeed()) { feedShooter(); } return !isEmpty();  },
                            // If there are no balls loaded, idle the shooter
                            p -> { if (isEmpty() || !isAimed() || !isAtSpeed()) { idleShooter(); } return isAtSpeed(); }
                    )
                    // TODO: Drive path
            );
        }
    }


}

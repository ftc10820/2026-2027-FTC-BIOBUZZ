package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test Auto Target", group="Test")
public class TestAutoTarget extends TestBaseOpMode {

    public void runOpMode() {
        initialize();
        // TODO: Initialize goalPositionX based on alliance color
        waitForStart();
        // Do stuff and drive
        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            // Check goal
                            p -> { detectGoal(); return false; },
                            // Set shooter speed
                            p -> { updateShooter(); return isAtSpeed(); },
                            // If there are balls loaded, feed the shooter
                            p -> { if (!isEmpty() && isAtSpeed()) { feedShooter(); } return !isEmpty();  },
                            // If there are no balls loaded, idle the shooter
                            p -> { if (isEmpty() || !isAtSpeed()) { idleShooter(); } return isAtSpeed(); }
                    )
                    // TODO: Drive path
            );
        }
    }


}

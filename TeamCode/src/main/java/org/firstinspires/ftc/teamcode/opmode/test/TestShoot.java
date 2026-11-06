package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PIDConstants;
import org.firstinspires.ftc.teamcode.opmode.test.TestBaseOpMode;

@Autonomous(name = "Test Auto Shoot", group = "Test")
public class TestShoot extends TestBaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        // TODO: Initialize goalPositionX based on alliance color
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                        // Set shooter speed
                        p -> { updateShooter(); return isAtSpeed(); },
                            new SleepAction(10) // wait 10 seconds
                    )
                );
        }
        // Turn off motor
        shooterMotor.setVelocity(0);

    }
}

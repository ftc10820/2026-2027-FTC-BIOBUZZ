package org.firstinspires.ftc.teamcode.opmode.test;

// import com.acmerobotics.roadrunner.ParallelAction;
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
            // Incoming parallel reporting/intake sequence retained but disabled.
            // Actions.runBlocking(
            //         new ParallelAction(
            //                 p -> { p.put("shooterRpm",PIDConstants.ticksToRpm(shooterMotor.getVelocity())); return false; },
            //                 new SequentialAction(
            //                     // Set shooter speed
            //                     p -> { updateShooter(); return isAtSpeed(); },
            //                     p -> { intakeMotor.setPower(1); return false; },
            //                     new SleepAction(10) // wait 10 seconds
            //                 )
            //         )
            //     );
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

package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

<<<<<<< HEAD
@Autonomous(name="Auto Red Loading", group = "Iterative OpMode")
=======
@Autonomous(name="Auto Red Loading", group = "Auto")
>>>>>>> refs/remotes/origin/master
public class AutoRedLoading extends InitOpMode{
    public void runOpMode() {
        initialize();

        // Set the robot's starting position
        Pose2d initialPose = new Pose2d(63, 12, Math.toRadians(180));

        waitForStart();
        if (opModeIsActive()) {

            Action path1 = drive.actionBuilder(initialPose)
                    // TODO: shoot
                    .splineTo(new Vector2d(52, 63), Math.PI)
                    .build();

            Actions.runBlocking(path1);
        }
    }
}


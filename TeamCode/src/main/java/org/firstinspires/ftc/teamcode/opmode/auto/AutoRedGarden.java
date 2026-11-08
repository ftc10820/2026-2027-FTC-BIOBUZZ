package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

<<<<<<< HEAD
@Autonomous(name = "Auto Red Gardem", group = "Iterative OpMode")
public class AutoRedGarden extends InitOpMode {

    @Override
=======
@Autonomous(name="Auto Red Garden", group = "Auto")
public class AutoRedGarden extends InitOpMode{
>>>>>>> refs/remotes/origin/master
    public void runOpMode() {

        initialize();

        // Set the robot's starting position
        Pose2d initialPose = new Pose2d(-63, 12, Math.toRadians(180));

        waitForStart();

        if (opModeIsActive()) {

            Action path1 = drive.actionBuilder(initialPose)
                    // TODO: shoot
                    .splineTo(new Vector2d(-58, 63), Math.PI)
                    // TODO: Intake
                    .splineTo(new Vector2d(-63, 63), Math.PI)
                    // TODO: off intake
                    .splineTo(new Vector2d(63, 63),Math.toRadians(0))
                    // TODO: aim
                    // TODO: shoot
                    .splineTo(new Vector2d(20, 63), Math.toRadians(0))
                    .build();

            Actions.runBlocking(path1);
        }
    }
}


package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

@Autonomous(name="Auto Test Op Mode", group = "Auto")
public class AutoTestOpMode extends InitOpMode{
    public void runOpMode() {
        initialize();
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        waitForStart();
        if(opModeIsActive()) {
            Action path1 = drive.actionBuilder(initialPose)
                    .splineTo(new Vector2d(0, 24), Math.toRadians(0))
                    .build();
            Actions.runBlocking(path1);
        }
    }
}

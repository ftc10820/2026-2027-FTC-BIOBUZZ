package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

@Autonomous(name="Basic: Iterative OpMode", group = "Iterative OpMode")
public class AutoRedGarden extends InitOpMode{
    public void runOpMode() {
        initialize();
        Pose2d initialPose = new Pose2d(-63, -12, Math.toRadians(180));
        waitForStart();
        while(opModeIsActive()) {
            Action path1 = drive.actionBuilder(initialPose)
                    .splineTo(new Vector2d(-58, 63), Math.PI)
                    .build();
        }
    }
}

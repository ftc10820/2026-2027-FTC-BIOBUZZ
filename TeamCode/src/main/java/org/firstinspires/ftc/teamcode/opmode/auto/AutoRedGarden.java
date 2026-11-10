package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

import java.util.Arrays;

@Autonomous(name = "Auto Red Garden", group = "Auto")
public class AutoRedGarden extends InitOpMode {
    // Inches and seconds. Each leg ends at rest, so the profile brakes before its target.
    private static final double MAX_SPEED = 30.0;
    private static final AccelConstraint MOVE_ACCEL = new ProfileAccelConstraint(-25.0, 30.0);

    @Override
    public void runOpMode() {
        initialize();

        // Field coordinates are (x, y), matching PinpointLocalizer.getPosX/getPosY.
        // Encoder direction reversal does not swap field axes. At -90 degrees,
        // forward is -Y and right is -X; after the clockwise turn, forward is -X.
        Pose2d initialPose = new Pose2d(61, -33, Math.toRadians(-90));
        VelConstraint moveSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED)
        ));

        // Every destination includes the 5-inch -Y correction. The fixed starting
        // pose makes the first forward leg 5 inches longer.
        Action path = drive.actionBuilder(initialPose)
                .waitSeconds(4.0)
                // Forward, then back, then right. Hold the starting heading.
                .strafeToConstantHeading(new Vector2d(61, -65), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                // Move 8 inches farther back, toward the blue hive.
                .strafeToConstantHeading(new Vector2d(61, -12), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(55, -13), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .turn(Math.toRadians(-90))
                // Now facing -X. Keep this heading through the loading/park zone.
                // Stop 2 inches earlier at the second shooting position.
                .strafeToConstantHeading(new Vector2d(-48, -13), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .waitSeconds(3.0)
                // Strafe left 4 inches farther (13 inches total), then move forward.
                .strafeToConstantHeading(new Vector2d(-48, -26), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-55, -26), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .waitSeconds(3.0)
                // Back, then left into the requested loading/park position.
                .strafeToConstantHeading(new Vector2d(-45, -26), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                // Finish the left strafe at the existing loading/park endpoint.
                .strafeToConstantHeading(new Vector2d(-45, -69), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();

        telemetry.addLine("Red Garden: place at (61, -33), facing -Y (-90 deg)");
        telemetry.addLine("Park target: (-45, -69), facing -X (180 deg)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // initialize() creates the localizer at (0, 0, 0). Re-anchor its latest
        // reading here so moving the robot during INIT does not offset the route.
        drive.updatePoseEstimate();
        drive.localizer.setPose(initialPose);

        try {
            Actions.runBlocking(packet -> opModeIsActive() && path.run(packet));
        } finally {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
    }
}


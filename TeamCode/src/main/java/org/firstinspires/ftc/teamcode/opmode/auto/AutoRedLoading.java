package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

import java.util.Arrays;

@Autonomous(name = "Auto Red Loading", group = "Auto")
public class AutoRedLoading extends InitOpMode {
    private static final double MAX_SPEED = 30.0;
    private static final AccelConstraint MOVE_ACCEL = new ProfileAccelConstraint(-25.0, 30.0);

    @Override
    public void runOpMode() {
        // initialize() holds the gate up/closed and stops the shooter and transfer.
        initialize();
        intakeMotor.setPower(0.0);
        initializeTurret();

        // Hold the chassis stopped during INIT; drive and spin up together at START.
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        Pose2d initialPose = new Pose2d(-60, -10, Math.toRadians(180));
        VelConstraint moveSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED)
        ));
        VelConstraint slowSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED / 2.0)
        ));
        ElapsedTime autoTimer = new ElapsedTime();
        // Facing -X (away from the hive), backward is +X. Hold this heading.
        Action path = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-59, -10), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();

        Action shooting = new SequentialAction(
                packet -> {
                    // These commands return immediately, so all three mechanisms move together.
                    // TeleOp's conversion includes the 2:1 reduction: 2300 shooter RPM = 4600 motor RPM.
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(2300.0 * 56.0 / 60.0);
                    }
                    transferMotor.setPower(1.0);
                    // TeleOp's helper applies the turret gearing; negative is clockwise from above.
                    moveTurretByDegrees(-180.0);
                    return false;
                },
                new SleepAction(3.0),
                packet -> {
                    stopperServo.setPosition(STOPPER_OPEN_POSITION);
                    return false;
                },
                // Start intake at 4 seconds; the stopper still opens at 3 seconds.
                new SleepAction(1.0),
                packet -> {
                    intakeMotor.setPower(1.0);
                    return false;
                }
        );
        Action openingActions = new ParallelAction(shooting, path);
        // Switch at seven seconds even if the initial one-inch move is not finished.
        Action opening = packet -> {
            if (autoTimer.seconds() >= 7.0) {
                intakeMotor.setPower(0.0);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }
            openingActions.run(packet);
            return true;
        };
        Action collection = drive.actionBuilder(new Pose2d(-59, -10, Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(-43, -10), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-43, -26), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .stopAndAdd(packet -> {
                    stopperServo.setPosition(STOPPER_OPEN_POSITION);
                    intakeMotor.setPower(1.0);
                    return false;
                })
                // Half the normal speed cap for this collection leg only.
                .strafeToConstantHeading(new Vector2d(-55, -24), slowSpeed, MOVE_ACCEL)
                .endTrajectory()
                .waitSeconds(3.0)
                .strafeToConstantHeading(new Vector2d(-45, -24), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-45, -57), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();
        // Lower the ramp at eight seconds without delaying the collection route.
        Action lowerRamp = packet -> {
            if (autoTimer.seconds() < 8.0) {
                return true;
            }
            flowerRampServo.setPosition(0.0);
            return false;
        };
        Action auto = new ParallelAction(new SequentialAction(opening, collection), lowerRamp);

        telemetry.addLine("Red Loading: start at (-60, -10), facing -X (180 deg)");
        telemetry.addLine("Gate opens at 3s; intake runs from 4 to 7s; collection route starts at 7s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        autoTimer.reset();

        // Anchor the localizer at START so movement during INIT does not offset the path.
        drive.updatePoseEstimate();
        drive.localizer.setPose(initialPose);

        try {
            Actions.runBlocking(packet -> {
                if (!opModeIsActive()) {
                    return false;
                }
                boolean running = auto.run(packet);
                if (shooterMotor != null) {
                    // 28 encoder ticks per motor turn * 2 motor turns per flywheel turn.
                    double shooterRpm = shooterMotor.getVelocity() * 60.0 / 56.0;
                    telemetry.addData("Shooter RPM", "%.0f", shooterRpm);
                    packet.put("Shooter RPM", shooterRpm);
                } else {
                    telemetry.addData("Shooter RPM", "Unavailable");
                }
                telemetry.update();
                return running;
            });
        } finally {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            intakeMotor.setPower(0.0);
            transferMotor.setPower(0.0);
            if (shooterMotor != null) {
                shooterMotor.setVelocity(0.0);
            }
        }
    }
}


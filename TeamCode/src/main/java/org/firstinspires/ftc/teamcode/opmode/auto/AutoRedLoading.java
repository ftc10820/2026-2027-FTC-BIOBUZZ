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

        // Keep the chassis still through INIT and the opening shooting sequence.
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        Pose2d initialPose = new Pose2d(-60, -42, Math.toRadians(180));
        VelConstraint moveSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED)
        ));
        VelConstraint slowSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED / 2.0)
        ));
        Action shooting = new SequentialAction(
                packet -> {
                    // These commands return immediately, so all three mechanisms move together.
                    // TeleOp's conversion includes the 2:1 reduction: 2300 shooter RPM = 4600 motor RPM.
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(2300.0 * 56.0 / 60.0);
                    }
                    transferMotor.setPower(1.0);
                    // TeleOp's helper applies the turret gearing; negative is clockwise from above.
                    moveTurretByDegrees(-135.0);
                    return false;
                },
                new SleepAction(4.0),
                packet -> {
                    stopperServo.setPosition(STOPPER_OPEN_POSITION);
                    intakeMotor.setPower(1.0);
                    return false;
                },
                // Feed from 4 to 6.5 seconds; keep the shooter and transfer running afterward.
                new SleepAction(2.5),
                packet -> {
                    intakeMotor.setPower(0.0);
                    return false;
                }
        );
        // Hold 180 degrees: backward is +X, right is +Y, and left is -Y.
        Action collection = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-50, -42), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .stopAndAdd(packet -> {
                    flowerRampServo.setPosition(0.0);
                    intakeMotor.setPower(1.0);
                    return false;
                })
                // The requested rightward leg also includes a five-inch +X offset.
                .strafeToConstantHeading(new Vector2d(-45, -24), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                // Half the normal speed cap for this collection leg only.
                .strafeToConstantHeading(new Vector2d(-55, -24), slowSpeed, MOVE_ACCEL)
                .endTrajectory()
                .waitSeconds(4.0)
                .strafeToConstantHeading(new Vector2d(-45, -24), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();
        Action finalStrafe = drive.actionBuilder(new Pose2d(-45, -24, Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(-45, -58), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();
        // This delay starts alongside the final strafe, not at the start of auto.
        Action raiseRampAndGate = new SequentialAction(
                new SleepAction(1.5),
                packet -> {
                    flowerRampServo.setPosition(0.5);
                    stopperServo.setPosition(STOPPER_CLOSED_POSITION);
                    return false;
                }
        );
        Action auto = new SequentialAction(
                shooting, collection, new ParallelAction(finalStrafe, raiseRampAndGate)
        );

        telemetry.addLine("Red Loading: start at (-60, -42), facing -X (180 deg)");
        telemetry.addLine("Gate opens and intake starts at 4s; driving starts at 6.5s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

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


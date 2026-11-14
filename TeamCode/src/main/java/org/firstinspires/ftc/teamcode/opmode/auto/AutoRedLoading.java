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

        // Hold the chassis stopped during INIT; drive and spin up together at START.
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        Pose2d initialPose = new Pose2d(-60, -10, Math.toRadians(180));
        VelConstraint moveSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED)
        ));
        // Facing -X (away from the hive), backward is +X. Hold this heading.
        Action path = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-48, -10), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();

        Action shooting = new SequentialAction(
                packet -> {
                    // These commands return immediately, so all three mechanisms move together.
                    // TeleOp's conversion includes the 2:1 reduction: 2200 shooter RPM = 4400 motor RPM.
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(2200.0 * 56.0 / 60.0);
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
                },
                // Intake runs from 4 to 6 seconds, independently of the drive action.
                new SleepAction(2.0),
                packet -> {
                    intakeMotor.setPower(0.0);
                    transferMotor.setPower(0.0);
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(0.0);
                    }
                    return false;
                }
        );
        Action auto = new ParallelAction(shooting, path);

        telemetry.addLine("Red Loading: start at (-60, -10), facing -X (180 deg)");
        telemetry.addLine("Back up to (-48, -10); gate opens at 3s; intake runs from 4 to 6s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        final long autoStartNs = System.nanoTime();

        // Anchor the localizer at START so movement during INIT does not offset the path.
        drive.updatePoseEstimate();
        drive.localizer.setPose(initialPose);

        try {
            Actions.runBlocking(packet -> {
                // End all actions at six seconds, even if the chassis has not reached its target.
                // The finally block stops the drive, intake, transfer, and shooter together.
                if (!opModeIsActive() || System.nanoTime() - autoStartNs >= 6_000_000_000L) {
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


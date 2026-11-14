package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
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

        // Keep the chassis stopped during INIT; shooting and driving begin at START.
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        Pose2d initialPose = new Pose2d(-58, -38, Math.toRadians(180));
        VelConstraint moveSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED)
        ));
        VelConstraint slowSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED / 2.0)
        ));
        // Hold 180 degrees: backward is +X, right is +Y, and left is -Y.
        Action approach = drive.actionBuilder(initialPose)
                .stopAndAdd(packet -> {
                    // Nonblocking commands: spin up and turn the turret as the first drive leg starts.
                    // 28 motor ticks/rev * 2:1 reduction: 2300 flywheel RPM = 4700 motor RPM.
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(2300.0 * 56.0 / 60.0);
                    }
                    transferMotor.setPower(1.0);
                    moveTurretByDegrees(-160.0);
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(-50, -38), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-50, -25), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .stopAndAdd(packet -> {
                    stopperServo.setPosition(STOPPER_OPEN_POSITION);
                    return false;
                })
                .waitSeconds(0.5)
                .stopAndAdd(packet -> {
                    intakeMotor.setPower(1.0);
                    return false;
                })
                .waitSeconds(3.0)
                .stopAndAdd(packet -> {
                    intakeMotor.setPower(0.0);
                    return false;
                })
                .strafeToConstantHeading(new Vector2d(-40, -25), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .stopAndAdd(packet -> {
                    flowerRampServo.setPosition(0.0);
                    intakeMotor.setPower(1.0);
                    return false;
                })
                // Only this forward collection leg uses half the normal speed cap (15 in/s).
                .strafeToConstantHeading(new Vector2d(-55, -25), slowSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();

        // Start the clock only after the first arrival at (-55, -25).
        Action oscillate = new Action() {
            private long startedNs;
            private boolean started;
            private boolean towardMinus54 = true;
            private Action leg;
            private Pose2d legStart = new Pose2d(-55, -25, Math.toRadians(180));

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    startedNs = System.nanoTime();
                    started = true;
                }
                if (System.nanoTime() - startedNs >= 4_500_000_000L) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    return false;
                }
                double targetX = towardMinus54 ? -54 : -55;
                if (leg == null) {
                    // Build a fresh action for each pass; completed actions cannot be reused.
                    leg = drive.actionBuilder(legStart)
                            .strafeToConstantHeading(new Vector2d(targetX, -25), slowSpeed, MOVE_ACCEL)
                            .endTrajectory()
                            .build();
                }
                if (!leg.run(packet)) {
                    legStart = new Pose2d(targetX, -25, Math.toRadians(180));
                    towardMinus54 = !towardMinus54;
                    leg = null;
                }
                return true;
            }
        };
        Action departure = new Action() {
            private Action route;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (route == null) {
                    // The timed oscillation can finish between endpoints. Continue from the measured pose.
                    drive.updatePoseEstimate();
                    route = drive.actionBuilder(drive.localizer.getPose())
                            .stopAndAdd(actionPacket -> {
                                intakeMotor.setPower(0.0);
                                transferMotor.setPower(0.0);
                                if (shooterMotor != null) {
                                    shooterMotor.setVelocity(0.0);
                                }
                                return false;
                            })
                            .strafeToConstantHeading(new Vector2d(-40, -25), moveSpeed, MOVE_ACCEL)
                            .endTrajectory()
                            .stopAndAdd(actionPacket -> {
                                // Raise the ramp and close the gate as the final left strafe starts.
                                flowerRampServo.setPosition(0.5);
                                stopperServo.setPosition(STOPPER_CLOSED_POSITION);
                                return false;
                            })
                            .strafeToConstantHeading(new Vector2d(-40, -65), moveSpeed, MOVE_ACCEL)
                            .endTrajectory()
                            .build();
                }
                return route.run(packet);
            }
        };
        Action auto = new SequentialAction(approach, oscillate, departure);

        telemetry.addLine("Red Loading: start at (-58, -38), facing -X (180 deg)");
        telemetry.addLine("Shoot at (-50, -25) for 3s; oscillate between X=-55 and -54 at Y=-25 for 4.5s; finish at (-40, -65)");
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


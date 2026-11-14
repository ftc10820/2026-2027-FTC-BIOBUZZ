package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
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

import org.firstinspires.ftc.teamcode.PIDConstants;
import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

import java.util.Arrays;

@Autonomous(name = "Auto Red Garden", group = "Auto")
public class AutoRedGarden extends InitOpMode {
    private static final double MAX_SPEED = 30.0;
    private static final AccelConstraint MOVE_ACCEL = new ProfileAccelConstraint(-25.0, 30.0);

    @Override
    public void runOpMode() {
        // initialize() holds the gate up/closed and stops the shooter and transfer.
        initialize();
        intakeMotor.setPower(0.0);
        initializeTurret();

        // Stay still during shooting, then drive from the original field starting pose.
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        Pose2d initialPose = new Pose2d(61, -33, Math.toRadians(-90));
        VelConstraint moveSpeed = new MinVelConstraint(Arrays.asList(
                drive.defaultVelConstraint,
                new TranslationalVelConstraint(MAX_SPEED)
        ));
        // Hold -90 degrees: forward is -Y and right is -X. Stop at each corner.
        Action path = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(61, -42), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-24, -42), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .strafeToConstantHeading(new Vector2d(-24, -60), moveSpeed, MOVE_ACCEL)
                .endTrajectory()
                .build();

        Action shooting = new SequentialAction(
                packet -> {
                    // These commands return immediately, so all three mechanisms move together.
                    // TeleOp's conversion includes the 2:1 reduction: 2400 shooter RPM = 4800 motor RPM.
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(PIDConstants.rpmToTicks(2400));
                    }
                    transferMotor.setPower(1.0);
                    // TeleOp's helper applies the turret gearing; negative is clockwise from above.
                    moveTurretByDegrees(-110.0);
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
                // Intake runs from 4 to 6.5 seconds; driving starts immediately afterward.
                new SleepAction(2.5),
                packet -> {
                    intakeMotor.setPower(0.0);
                    transferMotor.setPower(0.0);
                    if (shooterMotor != null) {
                        shooterMotor.setVelocity(0.0);
                    }
                    return false;
                }
        );
        Action auto = new SequentialAction(shooting, path);

        telemetry.addLine("Red Garden: start at (61, -33), facing -Y (-90 deg)");
        telemetry.addLine("Drive at 6.5s: (61, -42) -> (-24, -42) -> (-24, -55)");
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


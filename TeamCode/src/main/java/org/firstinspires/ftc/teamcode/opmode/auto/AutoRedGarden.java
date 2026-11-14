package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PIDConstants;
import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

@Autonomous(name = "Auto Red Garden", group = "Auto")
public class AutoRedGarden extends InitOpMode {
    @Override
    public void runOpMode() {
        // initialize() holds the gate up/closed and stops the shooter and transfer.
        initialize();
        intakeMotor.setPower(0.0);
        initializeTurret();

        // This autonomous only shoots; keep the chassis stopped throughout.
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

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
                new SleepAction(5.0),
                packet -> {
                    intakeMotor.setPower(0.0);
                    return false;
                }
        );

        telemetry.addLine("Red Garden: stationary shooting only");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        try {
            Actions.runBlocking(packet -> {
                if (!opModeIsActive()) {
                    return false;
                }
                boolean running = shooting.run(packet);
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


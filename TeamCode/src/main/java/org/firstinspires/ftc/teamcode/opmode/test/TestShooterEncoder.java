package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/** Isolates shooter encoder feedback from the transfer and the rest of TeleOp. */
@TeleOp(name = "Shooter Encoder Test", group = "FTC10820")
public class TestShooterEncoder extends LinearOpMode {
    // Same encoder assumption as TeleOp: 28 ticks per motor turn, external 2:1 reduction.
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_TURNS_PER_FLYWHEEL_TURN = 2.0;
    private static final double TICKS_PER_FLYWHEEL_REV =
            MOTOR_TICKS_PER_REV * MOTOR_TURNS_PER_FLYWHEEL_TURN;
    private static final double MAX_FLYWHEEL_RPM = 6000.0 / MOTOR_TURNS_PER_FLYWHEEL_TURN;

    @Override
    public void runOpMode() throws InterruptedException {
        // Do not call the shared robot initializer or run the transfer alongside the shooter.
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            motor.setPower(0.0);
        }
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMotorDisable();

        try {
            // Read the active settings for diagnosis; do not override or tune them.
            PIDFCoefficients activePidf =
                    shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            double selectedRpm = 1750.0;
            while (opModeInInit()) {
                telemetry.addLine("INIT: motor disabled. Turn flywheel by hand; watch encoder ticks.");
                telemetry.addLine("One flywheel turn should change ticks by about 56 (either sign).");
                showTelemetry(shooter, activePidf, false, selectedRpm);
                sleep(20);
            }
            if (isStopRequested()) {
                return;
            }

            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocity(0.0);
            boolean running = false;
            boolean bumperWasPressed = gamepad1.right_bumper;
            boolean upWasPressed = gamepad2.dpad_up;
            boolean downWasPressed = gamepad2.dpad_down;
            while (opModeIsActive()) {
                boolean bumperPressed = gamepad1.right_bumper;
                boolean upPressed = gamepad2.dpad_up;
                boolean downPressed = gamepad2.dpad_down;
                if (bumperPressed && !bumperWasPressed) {
                    running = !running;
                }
                if (upPressed && !upWasPressed) {
                    selectedRpm = Math.min(MAX_FLYWHEEL_RPM, selectedRpm + 250.0);
                }
                if (downPressed && !downWasPressed) {
                    selectedRpm = Math.max(250.0, selectedRpm - 250.0);
                }
                bumperWasPressed = bumperPressed;
                upWasPressed = upPressed;
                downWasPressed = downPressed;

                shooter.setVelocity(running ? selectedRpm * TICKS_PER_FLYWHEEL_REV / 60.0 : 0.0);
                telemetry.addLine("G1 right bumper: shooter on/off. G2 D-pad: +/-250 flywheel RPM.");
                telemetry.addLine("Transfer stays OFF. Do not turn flywheel by hand after START.");
                showTelemetry(shooter, activePidf, running, selectedRpm);
                sleep(20);
            }
        } finally {
            shooter.setPower(0.0);
            shooter.setMotorDisable();
        }
    }

    private void showTelemetry(DcMotorEx shooter, PIDFCoefficients activePidf,
                               boolean running, double selectedRpm) {
        double measuredTicksPerSecond = shooter.getVelocity();
        telemetry.addData("Shooter", running ? "ON" : "OFF");
        telemetry.addData("Flywheel RPM: selected / actual", "%.0f / %.0f", selectedRpm,
                measuredTicksPerSecond * 60.0 / TICKS_PER_FLYWHEEL_REV);
        telemetry.addData("Ticks/s: commanded / measured", "%.1f / %.1f",
                running ? selectedRpm * TICKS_PER_FLYWHEEL_REV / 60.0 : 0.0,
                measuredTicksPerSecond);
        telemetry.addData("Encoder ticks", shooter.getCurrentPosition());
        telemetry.addData("Mode", shooter.getMode());
        telemetry.addData("Active velocity PIDF (read only)", activePidf);
        telemetry.addData("Configured motor", shooter.getMotorType().getName());
        telemetry.addData("Configured motor ticks/rev / max RPM", "%.1f / %.0f",
                shooter.getMotorType().getTicksPerRev(), shooter.getMotorType().getMaxRPM());
        telemetry.addData("Motor connection", shooter.getConnectionInfo());
        telemetry.addData("Motor / encoder port", shooter.getPortNumber());
        telemetry.update();
    }
}

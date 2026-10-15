package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public abstract class InitOpMode extends LinearOpMode {

    private static final double SHOOTER_TARGET_RPM = 2500.0;
    // The 6000 RPM, 1:1 goBILDA Yellow Jacket has 28 ticks per output-shaft revolution.
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    protected static final double SHOOTER_TARGET_TICKS_PER_SECOND =
            SHOOTER_TARGET_RPM * SHOOTER_TICKS_PER_REV / 60.0;

    protected DcMotor intakeMotor;
    public MecanumDrive drive;

    // Optional until installed and configured. Check for null before using it.
    protected DcMotorEx shooterMotor;

    public void initialize() {

        // Match the drive motor names used by Road Runner MecanumDrive.
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        shooterMotor = hardwareMap.tryGet(DcMotorEx.class, "shooterMotor");
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            // Requires the encoder cable in the same numbered Hub port as motor power.
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor.setVelocity(0.0);
        }

        // Based on the forward test; verify each name maps to the correct physical wheel.

        System.out.println("Drive motors initialized");
        telemetry.addLine("Drive motors initialized");
        telemetry.update();

    }
}

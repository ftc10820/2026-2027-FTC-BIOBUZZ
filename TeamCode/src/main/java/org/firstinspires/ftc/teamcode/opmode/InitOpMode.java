package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class InitOpMode extends LinearOpMode {

    private static final double SHOOTER_TARGET_RPM = 3000.0;
    // The 6000 RPM, 1:1 goBILDA Yellow Jacket has 28 ticks per output-shaft revolution.
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    protected static final double SHOOTER_TARGET_TICKS_PER_SECOND =
            SHOOTER_TARGET_RPM * SHOOTER_TICKS_PER_REV / 60.0;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor intakeMotor;

    // Optional until installed and configured. Check for null before using it.
    protected DcMotorEx shooterMotor;

    public void initialize() {

        // Match the drive motor names used by Road Runner MecanumDrive.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Brake the drive motors whenever joystick input commands zero power.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        System.out.println("Drive motors initialized");
        telemetry.addLine("Drive motors initialized");
        telemetry.update();

    }
}

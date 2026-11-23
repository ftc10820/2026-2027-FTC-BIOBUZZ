package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testTeleop", group = "Test")
public class TestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Use the same wheel directions as the regular TeleOps.
        configureMotor(leftFront, DcMotor.Direction.REVERSE);
        configureMotor(rightFront, DcMotor.Direction.FORWARD);
        configureMotor(leftBack, DcMotor.Direction.FORWARD);
        configureMotor(rightBack, DcMotor.Direction.REVERSE);

        telemetry.addLine("G1 left stick: forward/back and strafe; right stick X: turn");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                // Match the regular TeleOps' stick signs and cubic response.
                double forward = Math.pow(-gamepad1.left_stick_y, 3);
                double left = Math.pow(-gamepad1.left_stick_x, 3);
                double turnLeft = Math.pow(-gamepad1.right_stick_x, 3);

                double leftFrontPower = forward - left - turnLeft;
                double rightFrontPower = forward + left + turnLeft;
                double leftBackPower = forward + left - turnLeft;
                double rightBackPower = forward - left + turnLeft;

                double maxPower = Math.max(1.0, Math.max(
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));

                leftFront.setPower(leftFrontPower / maxPower);
                rightFront.setPower(rightFrontPower / maxPower);
                leftBack.setPower(leftBackPower / maxPower);
                rightBack.setPower(rightBackPower / maxPower);
                idle();
            }
        } finally {
            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);
        }
    }

    private static void configureMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drivetrain Motor Test OpMode", group = "Test")
public class DrivetrainMotorTestOpMode extends LinearOpMode {
    private static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Match the directions used by MecanumDrive so positive power means robot-forward.
        configureMotor(leftFront, DcMotor.Direction.REVERSE);
        configureMotor(rightFront, DcMotor.Direction.FORWARD);
        configureMotor(leftBack, DcMotor.Direction.FORWARD);
        configureMotor(rightBack, DcMotor.Direction.REVERSE);

        telemetry.addLine("G1 D-pad up/down: leftFront forward/back");
        telemetry.addLine("G1 D-pad right/left: rightFront forward/back");
        telemetry.addLine("G1 Y/A: leftBack forward/back");
        telemetry.addLine("G1 B/X: rightBack forward/back");
        telemetry.addLine("Press the same button again to stop that motor.");
        telemetry.update();

        boolean upWasPressed = false;
        boolean downWasPressed = false;
        boolean leftWasPressed = false;
        boolean rightWasPressed = false;
        boolean yWasPressed = false;
        boolean aWasPressed = false;
        boolean bWasPressed = false;
        boolean xWasPressed = false;

        waitForStart();
        try {
            while (opModeIsActive()) {
                if (gamepad1.dpad_up && !upWasPressed) {
                    toggleMotor(leftFront, TEST_POWER);
                } else if (gamepad1.dpad_down && !downWasPressed) {
                    toggleMotor(leftFront, -TEST_POWER);
                }
                if (gamepad1.dpad_left && !leftWasPressed) {
                    toggleMotor(rightFront, -TEST_POWER);
                } else if (gamepad1.dpad_right && !rightWasPressed) {
                    toggleMotor(rightFront, TEST_POWER);
                }
                if (gamepad1.y && !yWasPressed) {
                    toggleMotor(leftBack, TEST_POWER);
                } else if (gamepad1.a && !aWasPressed) {
                    toggleMotor(leftBack, -TEST_POWER);
                }
                if (gamepad1.b && !bWasPressed) {
                    toggleMotor(rightBack, TEST_POWER);
                } else if (gamepad1.x && !xWasPressed) {
                    toggleMotor(rightBack, -TEST_POWER);
                }

                upWasPressed = gamepad1.dpad_up;
                downWasPressed = gamepad1.dpad_down;
                leftWasPressed = gamepad1.dpad_left;
                rightWasPressed = gamepad1.dpad_right;
                yWasPressed = gamepad1.y;
                aWasPressed = gamepad1.a;
                bWasPressed = gamepad1.b;
                xWasPressed = gamepad1.x;

                telemetry.addData("leftFront power", "%.1f", leftFront.getPower());
                telemetry.addData("rightFront power", "%.1f", rightFront.getPower());
                telemetry.addData("leftBack power", "%.1f", leftBack.getPower());
                telemetry.addData("rightBack power", "%.1f", rightBack.getPower());
                telemetry.update();
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

    private static void toggleMotor(DcMotor motor, double requestedPower) {
        motor.setPower(motor.getPower() == requestedPower ? 0.0 : requestedPower);
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: TestTeleOp", group="FTC10820")
public class TestOpMode extends InitOpMode{

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) frontRightDrive.setPower(1.0);
            if (gamepad1.dpad_down) frontRightDrive.setPower(0.0);

            if (gamepad1.dpad_left) frontLeftDrive.setPower(1.0);
            if (gamepad1.dpad_right) frontLeftDrive.setPower(0.0);

            if (gamepad2.dpad_up) backRightDrive.setPower(1.0);
            if (gamepad2.dpad_down) backRightDrive.setPower(0.0);

            if (gamepad2.dpad_left) backLeftDrive.setPower(1.0);
            if (gamepad2.dpad_right) backLeftDrive.setPower(0.0);

            if (gamepad2.left_bumper) intakeMotor.setPower(1.0);
            if (gamepad2.right_bumper) intakeMotor.setPower(0.0);
        }
        frontRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: TestTeleOp", group="FTC10820")
public class TestOpMode extends InitOpMode{

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) drive.rightFront.setPower(1.0);
            if (gamepad1.dpad_down) drive.rightFront.setPower(0.0);

            if (gamepad1.dpad_left) drive.leftFront.setPower(1.0);
            if (gamepad1.dpad_right) drive.leftFront.setPower(0.0);

            if (gamepad2.dpad_up) drive.rightBack.setPower(1.0);
            if (gamepad2.dpad_down) drive.rightBack.setPower(0.0);

            if (gamepad2.dpad_left) drive.leftBack.setPower(1.0);
            if (gamepad2.dpad_right) drive.leftBack.setPower(0.0);

            if (gamepad2.left_bumper) intakeMotor.setPower(1.0);
            if (gamepad2.right_bumper) intakeMotor.setPower(0.0);
        }
        drive.rightFront.setPower(0.0);
        drive.leftFront.setPower(0.0);
        drive.rightBack.setPower(0.0);
        drive.leftBack.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot: TestTeleOp", group="FTC10820")
public class TestOpMode extends InitOpMode{

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) rightFront.setPower(1.0);
            if (gamepad1.dpad_down) rightFront.setPower(0.0);

            if (gamepad1.dpad_left) leftFront.setPower(1.0);
            if (gamepad1.dpad_right) leftFront.setPower(0.0);

            if (gamepad2.dpad_up) rightBack.setPower(1.0);
            if (gamepad2.dpad_down) rightBack.setPower(0.0);

            if (gamepad2.dpad_left) leftBack.setPower(1.0);
            if (gamepad2.dpad_right) leftBack.setPower(0.0);

            if (gamepad2.left_bumper) intakeMotor.setPower(1.0);
            if (gamepad2.right_bumper) intakeMotor.setPower(0.0);
        }
        rightFront.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftBack.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}

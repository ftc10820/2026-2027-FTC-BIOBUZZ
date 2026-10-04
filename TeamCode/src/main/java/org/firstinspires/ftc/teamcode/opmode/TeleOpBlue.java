package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp Blue", group="FTC10820")
public class TeleOpBlue extends InitOpMode {
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            while (opModeIsActive()) {
                // Gamepad 1: left stick drives and strafes; right stick turns.
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double frontLeftPower = forward + strafe + turn;
                double frontRightPower = forward - strafe - turn;
                double backLeftPower = forward - strafe + turn;
                double backRightPower = forward + strafe - turn;

                // Scale all wheels together so no power exceeds -1 to 1.
                double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
                maxPower = Math.max(maxPower, Math.abs(frontRightPower));
                maxPower = Math.max(maxPower, Math.abs(backLeftPower));
                maxPower = Math.max(maxPower, Math.abs(backRightPower));

                frontLeftDrive.setPower(frontLeftPower / maxPower);
                frontRightDrive.setPower(frontRightPower / maxPower);
                backLeftDrive.setPower(backLeftPower / maxPower);
                backRightDrive.setPower(backRightPower / maxPower);

                // Hold the right trigger for full intake power; release to stop.
                intakeMotor.setPower(gamepad1.right_trigger > 0.0 ? 1.0 : 0.0);
            }
        } finally {
            frontLeftDrive.setPower(0.0);
            frontRightDrive.setPower(0.0);
            backLeftDrive.setPower(0.0);
            backRightDrive.setPower(0.0);
            intakeMotor.setPower(0.0);
        }
    }
}

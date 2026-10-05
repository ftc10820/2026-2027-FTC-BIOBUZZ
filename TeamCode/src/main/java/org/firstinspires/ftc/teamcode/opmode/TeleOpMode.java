package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

public abstract class TeleOpMode extends InitOpMode {

    public void setPoseFromAprilTag() {
        // TODO: Try to scan an AprilTag and reset the drive pose from it
    }

    public void setPoseInRedGardenCorner() {
        // TODO: Move the robot to the red garden corner and set the drive pose
    }

    public void setPoseInRedLoadingCorner() {
        // TODO: Move the robot to the red loading corner and set the drive pose
    }

    public void setPoseInBlueGardenCorner() {
        // TODO: Move the robot to the blue garden corner and set the drive pose
    }

    public void setPoseInBlueLoadingCorner() {
        // TODO: Move the robot to the blue loading corner and set the drive pose
    }

    public void doRoadRunnerDrive() {
        // Set the drivetrain with Road Runner according to the controls
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

        drive.updatePoseEstimate();
    }

    public void doManualDrive() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        drive.updatePoseEstimate();
    }

    public void doManualIntake() {
        // FTC RobotTeleopPOV_Linear motor on/off pattern, adapted to our intake.
        if (gamepad1.right_trigger > 0.0)
            intakeMotor.setPower(1.0);
        else
            intakeMotor.setPower(0.0);

    }
}

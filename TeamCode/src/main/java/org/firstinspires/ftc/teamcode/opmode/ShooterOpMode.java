package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.PIDConstants;

@TeleOp(name = "PID Tuning OpMode")
public class ShooterOpMode extends InitOpMode {

    private FtcDashboard dashboard;

    @Override
    public void runOpMode () {
        initialize();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(PIDConstants.KP,PIDConstants.KI,PIDConstants.KD,PIDConstants.KF));

        while (opModeIsActive()) {
            double currentVelocity = shooterMotor.getVelocity();
            double error = PIDConstants.TARGET_VELOCITY - currentVelocity;

            // Use the built-in motor PIDF controller to set the velocity
            shooterMotor.setVelocity(PIDConstants.TARGET_VELOCITY);

            telemetry.addData("Target Velocity", PIDConstants.TARGET_VELOCITY);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Error",error);
            telemetry.update();

        }
    }
}

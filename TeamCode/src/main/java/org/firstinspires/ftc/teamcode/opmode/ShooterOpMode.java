package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDConstants;

@TeleOp(name = "PID Tuning OpMode")
public class ShooterOpMode extends InitOpMode {

    private FtcDashboard dashboard;

    @Override
    public void runOpMode () {
        initialize();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            double currentVelocity = shooterMotor.getVelocity();
            double error = PIDConstants.TARGET_VELOCITY - currentVelocity;

            double output = (error * PIDConstants.KP);
            shooterMotor.setPower(output);

            telemetry.addData("Target Velocity", PIDConstants.TARGET_VELOCITY);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Error",error);
            telemetry.update();

        }
    }
}

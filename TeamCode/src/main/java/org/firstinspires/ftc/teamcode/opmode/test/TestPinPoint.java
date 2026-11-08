package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

@Autonomous(name="Test Pinpoint", group="Test")
public class TestPinPoint extends InitOpMode {
    public void runOpMode () {
        GoBildaPinpointDriver pinpoint1 = hardwareMap.get (GoBildaPinpointDriver.class,"pinpoint");
        waitForStart();
        while (opModeIsActive()) {
            StringBuffer buffer = new StringBuffer();
            buffer.append ("1: ");
            pinpoint1.update();
            buffer.append (pinpoint1.getEncoderX());
            telemetry.addLine(buffer.toString());
            telemetry.update();
        }
    }
}

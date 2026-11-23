package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.InitOpMode;

@Autonomous(name = "Auto Blue Garden", group = "Auto")
public class AutoBlueGarden extends InitOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("No autonomous actions configured");
        telemetry.update();
        waitForStart();
    }
}

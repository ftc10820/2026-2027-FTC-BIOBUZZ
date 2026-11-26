package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robot: TestVision", group="FTC10820")
public class VisionTestOpMode extends InitOpMode {
    @Override
    public void runOpMode() {
        initialize(PIPE_ANY_HIVE);
        vision.startStatusPolling();
        try {
            waitForStart();
            while (opModeIsActive()) {
                vision.update();
                vision.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            vision.stop();
        }
    }
}

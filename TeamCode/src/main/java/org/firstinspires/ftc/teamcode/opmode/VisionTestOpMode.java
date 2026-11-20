package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Autonomous(name="Robot: TestVision", group="FTC10820")
public class VisionTestOpMode extends InitOpMode{
    @Override
    public void runOpMode(){
        initialize();
        initializeLimelight();
        waitForStart();
        while(opModeIsActive()){
        List<LLResultTypes.FiducialResult> tags = findAprilTags();
        for (LLResultTypes.FiducialResult r : tags){

        }
        }

    }
}
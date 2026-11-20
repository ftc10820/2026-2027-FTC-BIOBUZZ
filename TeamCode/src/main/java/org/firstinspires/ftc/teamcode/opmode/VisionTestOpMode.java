package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
            telemetry.addLine ("=== Robot Status ===");
        for (LLResultTypes.FiducialResult r : tags){
            Pose3D pose =  r.getRobotPoseFieldSpace();
            telemetry.addData ("tag"+ r.getFiducialId(),pose.toString());
        }
        telemetry.update();
        }

    }
}
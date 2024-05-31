package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.detection.CenterstagePipeline;
import org.firstinspires.ftc.teamcode.detection.RedDetection1;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class wompwomp extends LinearOpMode {
    RedDetection1 pipeline = new RedDetection1();
    OpenCvInternalCamera camera;

    @Override
    public void runOpMode() throws InterruptedException{

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.update();
        }
    }
}

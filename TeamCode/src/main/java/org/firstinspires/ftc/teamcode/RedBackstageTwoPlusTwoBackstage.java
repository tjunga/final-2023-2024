package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.detection.RedDetection1;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Disabled
@Autonomous
public class RedBackstageTwoPlusTwoBackstage extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    RedDetection1 pipeline = new RedDetection1();
    @Override
    public void runOpMode() {
        initializeCamera();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        Pose2d startPose = new Pose2d(12,-60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        TrajectorySequence purple = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .lineTo(new Vector2d(12,-10))
                .build();
        TrajectorySequence yellow = drive.trajectorySequenceBuilder(purple.end())
                .strafeLeft(28)
                .splineToLinearHeading(new Pose2d(43,-33, Math.toRadians(0)), Math.toRadians(355))
                .build();
        TrajectorySequence cycle = drive.trajectorySequenceBuilder(yellow.end())
                .lineToLinearHeading(new Pose2d(24,-58.5, Math.toRadians(180)))
                .forward(72)
                .splineToConstantHeading(new Vector2d(-40, -42), Math.toRadians(355))
                .build();
        TrajectorySequence whitescore = drive.trajectorySequenceBuilder(cycle.end())
                .splineToLinearHeading(new Pose2d(-41,-58.8,Math.toRadians(0)), Math.toRadians(359))
                .lineTo(new Vector2d(47.1,-58.5))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(whitescore.end())
                .strafeRight(1)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .strafeLeft(1)
                .build();





        //position 2


        while(opModeInInit()) {
            mech.closeClaws();
            telemetry.addData("time to cook", pipeline.getPosition());
        }
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.addData("area", pipeline.getArea());
            telemetry.update();
            int position = pipeline.getPosition();
            mech.closeClaws();
            wait(500);
            drive.followTrajectorySequence(purple);
            mech.purplePosition();
            drive.followTrajectorySequence(yellow);
            mech.autoscoringPosition();
            wait(500);
            mech.storePosition();
            drive.followTrajectorySequence(cycle);
            mech.pickupwhitePixelB();
            wait(500);
            mech.resetSlide();
            drive.followTrajectorySequence(whitescore);
            mech.openClaws();
            drive.followTrajectorySequence(park1);
            mech.zeroPosition();
            drive.followTrajectorySequence(park2);


        }


    }
    public void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

    }
    public void wait(int ms) {
        ElapsedTime t = new ElapsedTime();
        t.startTime();
        while (t.milliseconds() < ms) {}
        t.reset();
    }
}
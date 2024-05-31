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
import org.firstinspires.ftc.teamcode.detection.BlueDetection;
import org.firstinspires.ftc.teamcode.detection.RedDetection1;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Disabled
@Autonomous
public class BlueBackstagePlus extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BlueDetection pipeline = new BlueDetection();
    @Override
    public void runOpMode() {
        initializeCamera();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        Pose2d startPose = new Pose2d(12,60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence yellow = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(43, 33, Math.toRadians(0)))
                .build();
        TrajectorySequence purple = drive.trajectorySequenceBuilder(yellow.end())
                .lineToLinearHeading(new Pose2d(12,10, Math.toRadians(90)))
                .build();
        TrajectorySequence whitecycle1 = drive.trajectorySequenceBuilder(purple.end())
                .lineToLinearHeading(new Pose2d(30,12, Math.toRadians(180)))
                .lineTo(new Vector2d(-30, 12))
                .lineToLinearHeading(new Pose2d(-40,8, Math.toRadians(180)))
                .build();
        TrajectorySequence whitescore1 = drive.trajectorySequenceBuilder(whitecycle1.end())
                .lineToLinearHeading(new Pose2d(36,8, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(43,33), Math.toRadians(0))
                .build();
        TrajectorySequence whitecycle2 = drive.trajectorySequenceBuilder(whitescore1.end())
                .strafeTo(new Vector2d(36,-8))
                .lineTo(new Vector2d(-30, 12))
                .lineToLinearHeading(new Pose2d(-40,8, Math.toRadians(180)))
                .build();
        TrajectorySequence whitescore2 = drive.trajectorySequenceBuilder(whitecycle2.end())
                .lineToLinearHeading(new Pose2d(36,8, Math.toRadians(0)))
                .strafeTo(new Vector2d(43,33))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder( whitescore2.end())
                .lineTo(new Vector2d(48,33))
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .forward(1)
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
            drive.followTrajectorySequence(yellow);
            mech.autoscoringPositionY();
            drive.followTrajectorySequence(purple);
            mech.purplePositionB();
            mech.resetSlide();
            drive.followTrajectorySequence(whitecycle1);
            mech.pickupwhitePixelB();
            mech.resetSlide();
            drive.followTrajectorySequence(whitescore1);
            mech.autoscoringPosition();
            mech.storePosition();
            drive.followTrajectorySequence(whitecycle2);
            mech.pickupwhitePixelB();
            mech.resetSlide();
            drive.followTrajectorySequence(whitescore2);
            mech.autoscoringPosition();
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
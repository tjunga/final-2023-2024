package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous
public class RedAudienceSide extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    RedDetection1 pipeline = new RedDetection1();
    public int position;
    @Override
    public void runOpMode() throws InterruptedException {
        initializeCamera();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();
        Pose2d startPose = new Pose2d(-34,-60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        TrajectorySequence purplepixel1 = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, -7, Math.toRadians(270)))
                .build();
        TrajectorySequence whitePixel1 = drive.trajectorySequenceBuilder(purplepixel1.end())
                .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel1 = drive.trajectorySequenceBuilder(whitePixel1.end())
                .lineTo(new Vector2d(40,-8))
                .lineToLinearHeading(new Pose2d(43, -24, Math.toRadians(0)))
                .build();
        TrajectorySequence park11 = drive.trajectorySequenceBuilder(yellowPixel1.end())
                .strafeRight(27)
                .build();
        TrajectorySequence park12 = drive.trajectorySequenceBuilder(park11.end())
                .forward(10)
                .waitSeconds(15)
                .build();
        TrajectorySequence nopark1 = drive.trajectorySequenceBuilder(yellowPixel1.end())
                .back(1)
                .build();
        TrajectorySequence nopark21 = drive.trajectorySequenceBuilder(nopark1.end())
                .strafeLeft(1)
                .build();
        //position2
        TrajectorySequence purplepixel = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, -7, Math.toRadians(270)))
                .build();
        TrajectorySequence whitePixel = drive.trajectorySequenceBuilder(purplepixel.end())
                .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(whitePixel.end())
                .lineTo(new Vector2d(40,-8))
                .lineToLinearHeading(new Pose2d(43, -33, Math.toRadians(0)))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .strafeRight(24)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .forward(10)
                .waitSeconds(15)
                .build();
        TrajectorySequence nopark = drive.trajectorySequenceBuilder(yellowPixel.end())
                .back(1)
                .build();
        TrajectorySequence nopark2 = drive.trajectorySequenceBuilder(nopark.end())
                .strafeLeft(1)
                .build();
        //position 3
        TrajectorySequence purplepixel3 = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, -7, Math.toRadians(270)))
                .build();
        TrajectorySequence whitePixel3 = drive.trajectorySequenceBuilder(purplepixel3.end())
                .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel3 = drive.trajectorySequenceBuilder(whitePixel3.end())
                .lineTo(new Vector2d(40,-8))
                .lineToLinearHeading(new Pose2d(41, -43, Math.toRadians(0)))
                .build();
        TrajectorySequence park13 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .strafeRight(22)
                .build();
        TrajectorySequence park23 = drive.trajectorySequenceBuilder(park13.end())
                .forward(10)
                .waitSeconds(15)
                .build();
        TrajectorySequence nopark3 = drive.trajectorySequenceBuilder(yellowPixel3.end())
                .back(1)
                .build();
        TrajectorySequence nopark23 = drive.trajectorySequenceBuilder(nopark3.end())
                .strafeLeft(1)
                .build();
        while(opModeInInit()) {
            mech.autocloseClaws();
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.addData("area", pipeline.getArea());
            telemetry.update();
            position = pipeline.getPosition();
        }
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){

            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.addData("area", pipeline.getArea());
            telemetry.update();
            mech.closeClaws();
            wait(1500);
            if (position == 1) {
                drive.followTrajectorySequence(purplepixel1);
                drive.turn(Math.toRadians(-45));
                mech.purplePosition();
                drive.followTrajectorySequence(whitePixel1);
                mech.pickupwhitePixel();
                drive.followTrajectorySequence(yellowPixel1);
                mech.autoscoringPosition();
                drive.followTrajectorySequence(nopark1);
                mech.zeroPosition();
                mech.closeClaws();
                drive.followTrajectorySequence(nopark21);
                break;
            }
            if (position == 2) {
                drive.followTrajectorySequence(purplepixel);
                mech.purplePosition();
                drive.followTrajectorySequence(whitePixel);
                mech.pickupwhitePixel();
                drive.followTrajectorySequence(yellowPixel);
                mech.autoscoringPosition();
                drive.followTrajectorySequence(nopark);
                mech.zeroPosition();
                mech.closeClaws();
                drive.followTrajectorySequence(nopark2);
                break;

            }
            if (position == 3) {
                drive.followTrajectorySequence(purplepixel3);
                drive.turn(Math.toRadians(28));
                mech.purplePosition3();
                drive.turn(Math.toRadians(-28));
                mech.resetSlide();
                drive.followTrajectorySequence(whitePixel3);
                mech.pickupwhitePixel();
                drive.followTrajectorySequence(yellowPixel3);
                mech.autoscoringPosition();
                drive.followTrajectorySequence(nopark3);
                mech.zeroPosition();
                mech.closeClaws();
                drive.followTrajectorySequence(nopark23);
                break;

            }
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
        while (t.milliseconds() < ms && opModeIsActive()) {}
        t.reset();
    }
}
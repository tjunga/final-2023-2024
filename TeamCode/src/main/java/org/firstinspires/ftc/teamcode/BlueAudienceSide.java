package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.detection.BlueDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous
public class BlueAudienceSide extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    BlueDetection pipeline = new BlueDetection();
    public int position;

    @Override
    public void runOpMode() {
        initializeCamera();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        Pose2d startPose = new Pose2d(-34,60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence purplepixel1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, 7, Math.toRadians(90)))
                .build();
        TrajectorySequence whitePixel1 = drive.trajectorySequenceBuilder(purplepixel1.end())
                .lineToLinearHeading(new Pose2d(-40, 8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel1 = drive.trajectorySequenceBuilder(whitePixel1.end())
                .lineTo(new Vector2d(40,8))
                .lineToLinearHeading(new Pose2d(43, 43, Math.toRadians(0)))
                .build();
        TrajectorySequence park11 = drive.trajectorySequenceBuilder(yellowPixel1.end())
                .strafeRight(24)
                .build();
        TrajectorySequence park12 = drive.trajectorySequenceBuilder(park11.end())
                .forward(10)
                .build();
        //pos 2
        TrajectorySequence purplepixel = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, 7, Math.toRadians(90)))
                .build();
        TrajectorySequence whitePixel = drive.trajectorySequenceBuilder(purplepixel.end())
                .lineToLinearHeading(new Pose2d(-40, 8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(whitePixel.end())
                .lineTo(new Vector2d(40,8))
                .lineToLinearHeading(new Pose2d(43, 35, Math.toRadians(0)))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .strafeRight(24)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .forward(10)
                .build();
        //pos 3
        TrajectorySequence purplepixel3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, 7, Math.toRadians(90)))
                .build();
        TrajectorySequence whitePixel3 = drive.trajectorySequenceBuilder(purplepixel3.end())
                .lineToLinearHeading(new Pose2d(-40, 8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel3 = drive.trajectorySequenceBuilder(whitePixel3.end())
                .lineTo(new Vector2d(40,8))
                .lineToLinearHeading(new Pose2d(43, 28, Math.toRadians(0)))
                .build();
        TrajectorySequence park13 = drive.trajectorySequenceBuilder(yellowPixel3.end())
                .strafeRight(16)
                .build();
        TrajectorySequence park23 = drive.trajectorySequenceBuilder(park13.end())
                .forward(10)
                .build();
        while(opModeInInit()) {
            double rightValue = ((DistanceSensor) mech.rSensor).getDistance(DistanceUnit.CM);
            double leftValue = ((DistanceSensor) mech.lSensor).getDistance(DistanceUnit.CM);
            if(rightValue < mech.clawThreshold) {
                mech.rC();
            }
            if(leftValue < mech.clawThreshold) {
                mech.lC();
            }            telemetry.addData("position: ", pipeline.getPosition());
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
            wait(5000);
            if (position == 1) {
                drive.followTrajectorySequence(purplepixel1);
                drive.turn(Math.toRadians(-35));
                mech.purplePositionB1();
                drive.followTrajectorySequence(whitePixel1);
                mech.pickupwhitePixelB();
                drive.followTrajectorySequence(yellowPixel1);
                mech.autoscoringPosition();
                drive.followTrajectorySequence(park11);
                mech.zeroPosition();
                mech.closeClaws();
                drive.followTrajectorySequence(park12);
                break;
            }
            if (position == 2) {
                drive.followTrajectorySequence(purplepixel);
                mech.purplePositionB();
                drive.followTrajectorySequence(whitePixel);
                mech.pickupwhitePixelB();
                drive.followTrajectorySequence(yellowPixel);
                mech.autoscoringPosition();
                drive.followTrajectorySequence(park1);
                mech.zeroPosition();
                mech.closeClaws();
                drive.followTrajectorySequence(park2);
                break;
            }
            if (position == 3 || position == 0) {
                drive.followTrajectorySequence(purplepixel3);
                drive.turn(Math.toRadians(45));
                mech.purplePositionB3();
                drive.followTrajectorySequence(whitePixel3);
                mech.pickupwhitePixelB();
                drive.followTrajectorySequence(yellowPixel3);
                mech.autoscoringPosition();
                drive.followTrajectorySequence(park13);
                mech.zeroPosition();
                mech.closeClaws();
                drive.followTrajectorySequence(park23);
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
        while (t.milliseconds() < ms) {}
        t.reset();
    }
}
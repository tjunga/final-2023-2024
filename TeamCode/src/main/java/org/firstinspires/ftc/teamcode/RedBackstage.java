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
import org.firstinspires.ftc.teamcode.detection.RedDetection1;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous
public class RedBackstage extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    RedDetection1 pipeline = new RedDetection1();
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
        Pose2d startPose = new Pose2d(12,-60, Math.toRadians(270));

       //pos 1
        TrajectorySequence start1 = drive.trajectorySequenceBuilder(startPose)
                .back(3)
                .build();
        TrajectorySequence purple1 = drive.trajectorySequenceBuilder(start1.end())
                .lineToLinearHeading(new Pose2d(12,-40, Math.toRadians(90)))
                .build();
        TrajectorySequence yellow1 = drive.trajectorySequenceBuilder(purple1.end())
                .back(17.5)
                .strafeRight(28)
                .splineToLinearHeading(new Pose2d(42.45,-28, Math.toRadians(0)), Math.toRadians(355))
                .build();
        TrajectorySequence park11 = drive.trajectorySequenceBuilder(yellow1.end())
                .strafeTo(new Vector2d(48,-58))
                .build();
        TrajectorySequence park21 = drive.trajectorySequenceBuilder(park11.end())
                .forward(5)
                .build();

        //pos 2
        drive.setPoseEstimate(startPose);
        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                .back(3)
                .build();
        TrajectorySequence purple = drive.trajectorySequenceBuilder(start.end())
                .lineToLinearHeading(new Pose2d(12,-38, Math.toRadians(90)))
                .build();
        TrajectorySequence yellow = drive.trajectorySequenceBuilder(purple.end())
                .back(19.5)
                .strafeRight(28)
                .splineToLinearHeading(new Pose2d(40.45,-33, Math.toRadians(0)), Math.toRadians(355))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(yellow.end())
                .strafeTo(new Vector2d(48,-58))
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .forward(5)
                .build();
        //pos 3
        TrajectorySequence start3 = drive.trajectorySequenceBuilder(startPose)
                .back(3)
                .build();
        TrajectorySequence purple3 = drive.trajectorySequenceBuilder(start3.end())
                .lineToLinearHeading(new Pose2d(12,-38, Math.toRadians(90)))
                .build();
        TrajectorySequence yellow3 = drive.trajectorySequenceBuilder(purple3.end())
                .back(19.5)
                .strafeRight(35)
                .splineToLinearHeading(new Pose2d(40.45,-38, Math.toRadians(0)), Math.toRadians(355))
                .build();
        TrajectorySequence park13 = drive.trajectorySequenceBuilder(yellow3.end())
                .strafeTo(new Vector2d(48,-58))
                .build();
        TrajectorySequence park23 = drive.trajectorySequenceBuilder(park13.end())
                .forward(5)
                .build();



        //position 2


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
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.addData("area", pipeline.getArea());
            telemetry.update();
            mech.closeClaws();
            wait(500);
            if(position == 1) {
                drive.followTrajectorySequence(start1);
                wait(500);
                drive.followTrajectorySequence(purple1);
                drive.turn(Math.toRadians(38));
                mech.purplePosition();
                wait(500);
                mech.scoringPosition();
                wait(100);
                mech.resetSlide();
                drive.turn(Math.toRadians(-38));
                drive.followTrajectorySequence(yellow1);
                mech.autoscoringPosition();
                mech.resetSlide();
                drive.followTrajectorySequence(park11);
                mech.zeroPosition();
                drive.followTrajectorySequence(park21);
                break;
            }
            if(position == 2) {
                drive.followTrajectorySequence(start);
                drive.followTrajectorySequence(purple);
                mech.purplePosition();
                mech.resetSlide();
                drive.followTrajectorySequence(yellow);
                mech.autoscoringPosition();
                mech.resetSlide();
                drive.followTrajectorySequence(park1);
                mech.zeroPosition();
                drive.followTrajectorySequence(park2);
                break;
            }
            if (position == 3) {
                drive.followTrajectorySequence(start3);
                mech.grabbingPosition();
                drive.followTrajectorySequence(purple3);
                drive.turn(Math.toRadians(-48.5));
                mech.purplePosition();
                wait(500);
                mech.scoringPosition();
                wait(500);
                mech.resetSlide();
                drive.turn(Math.toRadians(48.5));
                drive.followTrajectorySequence(yellow3);
                mech.autoscoringPosition();
                mech.resetSlide();
                drive.followTrajectorySequence(park13);
                mech.zeroPosition();
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
        while(t.milliseconds() < ms && opModeIsActive()) {}
        t.reset();
    }
}
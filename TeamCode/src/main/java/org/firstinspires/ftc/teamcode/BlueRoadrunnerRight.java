package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.detection.BlueDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@Autonomous
public class BlueRoadrunnerRight extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-34,60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double pixelLocation = 90;
        double scoringPosition = 35;



        TrajectorySequence purplepixel = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(5)
                .back(1)
                .lineToLinearHeading(new Pose2d(-34, 7, Math.toRadians(pixelLocation)))
                .build();

        TrajectorySequence whitePixel = drive.trajectorySequenceBuilder(purplepixel.end())
                .lineToLinearHeading(new Pose2d(-40, 8, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(whitePixel.end())
                .back(85)
                .splineToLinearHeading(new Pose2d(43, scoringPosition, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .strafeRight(24)
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(park1.end())
                .forward(10)
                .build();


        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.addData("area", pipeline.getArea());
            telemetry.update();
        }
        mech.closeClaws();

        if (isStopRequested()) return;
            if (pipeline.getPosition() == 1) {
                pixelLocation = 45;
            }
            if (pipeline.getPosition() == 2) {
                pixelLocation = 90;
            }
            if (pipeline.getPosition() == 3) {
                pixelLocation = 135;
            }
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
}
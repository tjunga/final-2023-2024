package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.drive.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class twoplusthree_red extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        Pose2d startPose = new Pose2d(-34,-60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        double pixelLocation = 270;


        TrajectorySequence purplepixel = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .splineToLinearHeading(new Pose2d(-36, -40, Math.toRadians(90)), Math.toRadians(0))
                .build();

        TrajectorySequence whitePixel = drive.trajectorySequenceBuilder(purplepixel.end())
                .lineToLinearHeading(new Pose2d(-40, -28, Math.toRadians(180)))
                .build();
        TrajectorySequence yellowPixel = drive.trajectorySequenceBuilder(whitePixel.end())
                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                .build();
        TrajectorySequence whitepixel2 = drive.trajectorySequenceBuilder(yellowPixel.end())
                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-40, -28, Math.toRadians(180)))
                .build();

        TrajectorySequence yellow2 = drive.trajectorySequenceBuilder(whitepixel2.end())
                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, -38, Math.toRadians(0)))
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(yellow2.end())
                .strafeLeft(10)
                .build();


        waitForStart();
        mech.closeClaws();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(purplepixel);
        mech.purplePosition();
        drive.followTrajectorySequence(whitePixel);
        mech.pickupwhitePixel();
        drive.followTrajectorySequence(yellowPixel);
        mech.autoscoringPosition();
        drive.followTrajectorySequence(whitepixel2);
        mech.pickupwhitePixel();
        drive.followTrajectorySequence(yellow2);
        mech.autoscoringPosition();
        mech.zeroPosition();
        mech.closeClaws();
        drive.followTrajectorySequence(park);

    }
}
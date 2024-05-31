package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.drive.trajectorysequence.TrajectorySequence;


@Config
@Disabled
@Autonomous
public class imLazy extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(this);
        Drivetrain drivetrain = new Drivetrain(this);
        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        Pose2d startPose = new Pose2d(34,11, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence lazy = drive.trajectorySequenceBuilder(startPose)
                .back(85)
                .build();
        TrajectorySequence lazy1 = drive.trajectorySequenceBuilder(lazy.end())
                .waitSeconds(3)
                .strafeRight(1)
                .build();
        waitForStart();
        mech.closeClaws();

        if (isStopRequested()) return;
        mech.storePosition();
        drive.followTrajectorySequence(lazy);
        mech.zeroPosition();
        drive.followTrajectorySequence(lazy1);

    }
}
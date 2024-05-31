package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class TestingAutoGrab extends LinearOpMode {
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mechanisms = new Mechanisms(this);
        drivetrain.initDrivetrain(hardwareMap);
        mechanisms.initMechanisms(hardwareMap);
        mechanisms.initEncoders();

        waitForStart();
        while (opModeInInit()) {
            mechanisms.openClaws();
        }
        while (opModeIsActive()) {
            mechanisms.axle1.setTargetPosition(4000);
            mechanisms.axle2.setTargetPosition(4000);
            mechanisms.axle1.setPower(1);
            mechanisms.axle2.setPower(1);
            mechanisms.axle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mechanisms.axle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wait(1000);
            mechanisms.setPivotGrab();
            wait(1500);
            mechanisms.autoGrabRed();
        }
    }
    public void wait(int ms) {
        ElapsedTime t = new ElapsedTime();
        t.startTime();
        while(t.milliseconds() < ms && opModeIsActive()) {}
        t.reset();
    }
}


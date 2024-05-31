package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTesting extends LinearOpMode {
    public static double a =  0.3;

    public void runOpMode(){
        Servo launcher = hardwareMap.get(Servo.class, "drone");
        //Servo right = hardwareMap.get(Servo.class, "right");
        //Drivetrain hazel = new Drivetrain(this, hardwareMap);
        waitForStart();
        while (opModeIsActive()){
//            if (gamepad1.right_bumper) {
//                double currentPosition = door.getPosition();
//                door.setPosition(currentPosition + 0.05);
//                telemetry.addData("door", door.getPosition());
//                telemetry.update();
//            } else if (gamepad1.left_bumper) {
//                double currentPosition = door.getPosition();
//                door.setPosition(currentPosition - 0.05);
//                telemetry.addData("door", door.getPosition());
//                telemetry.update();
//            }
            //door.setPosition(0.5);
            if(gamepad2.a){
                launcher.setPosition(a);
            }
/*
            for (int i = 1; i <= 20; i++) {
                left.setPosition(left.getPosition() + 0.05);
                //right.setPosition(left.getPosition() + 0.05);
                hazel.wait(1000);
                telemetry.addData("left", left.getPosition());
                //telemetry.addData("right", right.getPosition());
                telemetry.update();
            }

 */


//            telemetry.addData("left", left.getPosition());
            //telemetry.addData("right", right.getPosition());
            if (gamepad1.right_bumper) {

            }
            telemetry.update();


        }
    }
}
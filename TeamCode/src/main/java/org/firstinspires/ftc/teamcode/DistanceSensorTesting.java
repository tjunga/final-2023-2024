package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class DistanceSensorTesting extends LinearOpMode {
    public static double threshold =  0.25;

    public void runOpMode(){
        Mechanisms mech = new Mechanisms(this);

        mech.initMechanisms(hardwareMap);
        mech.initEncoders();
        //Servo right = hardwareMap.get(Servo.class, "right");
        //Drivetrain hazel = new Drivetrain(this, hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (mech.rSensor instanceof DistanceSensor) {
                ((DistanceSensor) mech.rSensor).getDistance(DistanceUnit.CM);
            }
            if (mech.lSensor instanceof DistanceSensor) {
                ((DistanceSensor) mech.lSensor).getDistance(DistanceUnit.CM);
            }
            if (mech.pivotSensor instanceof DistanceSensor) {
                ((DistanceSensor) mech.pivotSensor).getDistance(DistanceUnit.CM);
            }
            if (((DistanceSensor) mech.rSensor).getDistance(DistanceUnit.CM) < threshold) {
                telemetry.addLine("Pixel Detected R");
            }
            else {
                telemetry.addLine("No Pixel R");
            }
            if (((DistanceSensor) mech.lSensor).getDistance(DistanceUnit.CM) < threshold) {
                telemetry.addLine("Pixel Detected L");
            }
            else {
                telemetry.addLine("No Pixel L");
            }

            if(gamepad2.a){
                double rightValue = ((DistanceSensor) mech.rSensor).getDistance(DistanceUnit.CM);
                double leftValue = ((DistanceSensor) mech.lSensor).getDistance(DistanceUnit.CM);

                if(rightValue < threshold) {
                    mech.rC();
                }
                if(leftValue < threshold) {
                    mech.lC();
                }
            }
/*          if(gamepad2.a) {
                double rightValue = ((DistanceSensor) mech.rSensor).getDistance(DistanceUnit.CM);
                double leftValue = ((DistanceSensor) mech.lSensor).getDistance(DistanceUnit.CM);
                if(rightValue < mech.clawThreshold) {
                    mech.rC();
                }
                if(leftValue < mech.clawThreshold) {
                    mech.lC();
                }
            }
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
            if (gamepad2.y) {
                mech.openClaws();
            }

            }
            telemetry.update();


        }
}

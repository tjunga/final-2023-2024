package org.firstinspires.ftc.teamcode.detection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class RedDetection1 extends OpenCvPipeline {
    int position = 0;
    double area = 0;
    double tl = 0;
    int xThreshold = 495;
    int areaThreshold = 8000;
// nigger
    Scalar lower = new Scalar(0, 150, 20);
    Scalar upper = new Scalar(25, 255, 255);
    Scalar red = new Scalar(255, 0, 0);

    Mat hsv = new Mat();
    Mat mask = new Mat();

    void convertHSV(Mat input){
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower, upper, mask);
    }

    @Override
    public void init(Mat firstFrame){
        convertHSV(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input){
        convertHSV(input);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Rect bounding_rect = new Rect();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        boolean teamElement= false;
        if(contours.size() != 0){
            for(Mat contour : contours){
                if(Imgproc.contourArea(contour) > areaThreshold){
                    teamElement = true;
                    bounding_rect = Imgproc.boundingRect(contour);
                    area = Imgproc.contourArea(contour);
                    tl = bounding_rect.tl().x;
                    Imgproc.rectangle(input, bounding_rect.tl(), bounding_rect.br(), red, 5);
                }
            }
        }
        if (!teamElement) {
            position = 3;
        } else {
            if(bounding_rect.tl().x < xThreshold){
                position = 1;
            } else if (bounding_rect.tl().x > xThreshold){
                position = 2;
            }
        }
        return input;
    }
    public int getPosition() {
        return position;
    }
    public double getArea() {
        return area;
    }
    public double getTL() {
        return tl;
    }
}

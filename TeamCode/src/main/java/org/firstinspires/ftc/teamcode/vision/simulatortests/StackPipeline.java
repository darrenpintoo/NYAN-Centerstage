package org.firstinspires.ftc.teamcode.vision.simulatortests;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class StackPipeline implements VisionProcessor {


    private double STACK_WIDTH = 2;
    private double STACK_HEIGHT = 2;

    public Scalar lowerBound = new Scalar(0, 117.6, 128.9); // new Scalar(25.5, 80.8, 131.8);
    public Scalar upperBound = new Scalar(17.0, 255, 255);// new Scalar(46.8, 255, 255);

    private Mat hsvMat       = new Mat();
    private Mat thresholdMat       = new Mat();
    private Mat contourMat = new Mat();

    private double lastCaptureTime = 0;


    private final ArrayList<MatOfPoint> listOfContours = new ArrayList<>();
    Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(3, 3));
    Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(3, 3));

    Telemetry telemetry;

    public StackPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        listOfContours.clear();
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        Core.inRange(frame, lowerBound, upperBound, frame);

        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_ERODE, kernel1);
        Imgproc.morphologyEx(frame, frame, Imgproc.MORPH_DILATE, kernel2);

        Imgproc.findContours(frame, listOfContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        MatOfPoint largestContour = new MatOfPoint();
        double largestContourArea = -1;

        if (listOfContours.size() > 0) {

            for (int i = 0; i < listOfContours.size(); i++) {

                MatOfPoint currentContour = listOfContours.get(i);
                double currentContourArea = Imgproc.contourArea(currentContour);
                if (currentContourArea < 50) {
                    continue;
                }

                if (currentContourArea > largestContourArea) {
                    largestContour = currentContour;
                    largestContourArea = currentContourArea;
                }

                Rect boundingBox = Imgproc.boundingRect(currentContour);
                Imgproc.rectangle(frame, boundingBox, new Scalar(255, 255 , 255));

            }


            Rect boundingBox = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(frame, boundingBox, new Scalar(0, 255 , 255));

            int centerXCoordinate = boundingBox.x + boundingBox.width / 2;
            int centerYCoordinate = boundingBox.y + boundingBox.height / 2;

            Imgproc.circle(frame, new Point(centerXCoordinate, centerYCoordinate), 3, new Scalar(255, 0, 0));

            double conversionPixelsToDegrees = CameraConstants.FrontCamera.fovXDeg / frame.size().width;

            // double linearDegreesErrorX = -(centerXCoordinate - (frame.size().width / 2)) * conversionPixelsToDegrees;
            double curvedDegreesErrorX = -Math.toDegrees(Math.atan2((centerXCoordinate - (frame.size().width / 2)), CameraConstants.FrontCamera.fx));
            double curvedDegreesErrorY = -Math.toDegrees(Math.atan2((centerYCoordinate - (frame.size().height / 2)), CameraConstants.FrontCamera.fy));

            double ratioX = STACK_WIDTH / boundingBox.width;
            double ratioY = STACK_HEIGHT / boundingBox.height;

            double depthX = ratioX * CameraConstants.fx;
            double depthY = ratioY * CameraConstants.fy;

            double rayDistance = Math.hypot(depthX, depthY); // true distance

            // <ignore>
            double hypotenuseY = rayDistance / Math.cos(Math.toRadians(curvedDegreesErrorY)); // have angle and adj, need hyp
            double hypotenuseX = Math.abs(rayDistance * Math.tan(Math.toRadians(curvedDegreesErrorX))); // have angle and adj, need opp

            double distanceToCamera = Math.cbrt(Math.pow(hypotenuseX, 3)  + Math.pow(hypotenuseY, 3) + Math.pow(rayDistance, 3)); // inaccurate
            // </ignore>


            if (this.telemetry != null) {
                telemetry.addData("X Degrees Error: ", curvedDegreesErrorX);
                telemetry.addData("Y Degrees Error: ", curvedDegreesErrorY);

                telemetry.addData("Depth (X): ", depthX);
                telemetry.addData("Depth (Y): ", depthY);

                telemetry.addData("Ray Distance: ", rayDistance);

                telemetry.addData("Hypotenuse Y: ", hypotenuseY);
                telemetry.addData("Hypotenuse X: ", hypotenuseX);
                telemetry.addData("Delay: ", captureTimeNanos - lastCaptureTime);
            }
            // t.addData("Distance: ", distanceToCamera);


        }


        telemetry.addLine("Contours: " + listOfContours.size());
        telemetry.update();

        lastCaptureTime = captureTimeNanos;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}

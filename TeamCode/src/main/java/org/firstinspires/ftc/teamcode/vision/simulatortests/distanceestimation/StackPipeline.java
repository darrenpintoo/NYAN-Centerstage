package org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
// import android.graphics.RectF;


import androidx.core.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
// import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
// import org.firstinspires.ftc.teamcode.vision.VisionUtilities;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation.CameraConstants;
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


    private double STACK_WIDTH = 3;
    private double STACK_HEIGHT = 2.5;

    public Scalar lowerBound = new Scalar(0, 160, 0); // new Scalar(25.5, 80.8, 131.8);
    public Scalar upperBound = new Scalar(255, 255, 255);// new Scalar(46.8, 255, 255);

    private Mat hsvMat       = new Mat();
    private Mat thresholdMat       = new Mat();
    private Mat contourMat = new Mat();

    private double lastCaptureTime = 0;
    // private Pose correctionPose = new Pose(0, 0, 0);
    private Rect boundingBox;
    // private final Paint borderPaint = new Paint();

    double strafeError;


    private final ArrayList<MatOfPoint> listOfContours = new ArrayList<>();
    Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(10, 10));
    Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(10, 10));

    Telemetry telemetry;

    public StackPipeline() {

        // borderPaint.setColor(Color.MAGENTA);
        // borderPaint.setStyle(Paint.Style.STROKE);
        // borderPaint.setStrokeWidth(2);


    }

    public StackPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        // borderPaint.setColor(Color.MAGENTA);
        // borderPaint.setStyle(Paint.Style.STROKE);
        // borderPaint.setStrokeWidth(2);

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        listOfContours.clear();

        thresholdMat = frame.submat(new Rect(0, 480 / 2, 640, 480 / 2));
        Imgproc.cvtColor(thresholdMat, thresholdMat, Imgproc.COLOR_RGB2HLS);

        Core.inRange(thresholdMat, lowerBound, upperBound, thresholdMat);

        Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_ERODE, kernel1);
        Imgproc.morphologyEx(thresholdMat, thresholdMat, Imgproc.MORPH_DILATE, kernel2);

        Imgproc.findContours(thresholdMat, listOfContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint largestContour = new MatOfPoint();
        double largestContourArea = -1;

        if (listOfContours.size() > 0) {

            for (int i = 0; i < listOfContours.size(); i++) {

                MatOfPoint currentContour = listOfContours.get(i);
                double currentContourArea = Imgproc.contourArea(currentContour);

                if (currentContourArea > largestContourArea) {
                    largestContour = currentContour;
                    largestContourArea = currentContourArea;
                }

                Rect boundingBox = Imgproc.boundingRect(currentContour);
                Imgproc.rectangle(thresholdMat, boundingBox, new Scalar(255, 255 , 255));

            }


            boundingBox = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(thresholdMat, boundingBox, new Scalar(0, 100 , 255), 1);
            Imgproc.rectangle(frame, boundingBox, new Scalar(0, 100 , 255), 5);

            int centerXCoordinate = boundingBox.x + boundingBox.width / 2;
            int centerYCoordinate = boundingBox.y + boundingBox.height / 2;

            Imgproc.circle(thresholdMat, new Point(centerXCoordinate, centerYCoordinate), 3, new Scalar(255, 0, 0));

            double conversionPixelsToDegrees = CameraConstants.FrontCamera.fovXDeg / frame.size().width;

            // double linearDegreesErrorX = -(centerXCoordinate - (frame.size().width / 2)) * conversionPixelsToDegrees;
            double curvedDegreesErrorX = -Math.toDegrees(Math.atan2((centerXCoordinate - (CameraConstants.FrontCamera.cx)), CameraConstants.FrontCamera.fx));
            double curvedDegreesErrorY = -Math.toDegrees(Math.atan2((centerYCoordinate - (CameraConstants.FrontCamera.cy)), CameraConstants.FrontCamera.fy));

            double ratioX = STACK_WIDTH / boundingBox.width;
            double ratioY = STACK_HEIGHT / boundingBox.height;

            double depthX = ratioX * CameraConstants.FrontCamera.fx;
            double depthY = ratioX * CameraConstants.FrontCamera.fx;

            double rayDistance = Math.hypot(depthX, depthY); // true distance

            // <ignore>
            double hypotenuseY = rayDistance / Math.cos(Math.toRadians(curvedDegreesErrorY)); // have angle and adj, need hyp
            double hypotenuseX = rayDistance * Math.tan(Math.toRadians(curvedDegreesErrorX)); // have angle and adj, need opp

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


                telemetry.addLine("Contours: " + listOfContours.size());
                telemetry.update();
            }

            strafeError = hypotenuseX;


            // t.addData("Distance: ", distanceToCamera);

            // correctionPose = new Pose(hypotenuseX, hypotenuseY, 0);

        }

        lastCaptureTime = captureTimeNanos;

        // thresholdMat.copyTo(frame);
        return thresholdMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {



        if (boundingBox != null) {
            // VisionUtilities.drawRectangle(canvas, boundingBox, scaleBmpPxToCanvasPx);
        }






    }

    public Pose getCorrection() {
        return new Pose(- this.getStrafeError(), 0, 0);
    }

    public double getStrafeError() {
        return MathUtils.clamp(strafeError, -3, 3);
    }

}

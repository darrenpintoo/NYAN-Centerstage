package org.firstinspires.ftc.teamcode.vision.simulatortests;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PropDetectionPipelineBlueClose implements VisionProcessor {

    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 117.6, 255);


    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    double redAmount1 = 0;
    double redAmount2 = 0;

    double redAmount3 = 0;
    private final double redThreshold = 3000;
    private volatile PlacementPosition placementPosition = PlacementPosition.CENTER;

    public PropDetectionPipelineBlueClose() {

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        maskedInputMat.release();
        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        // Define the coordinates of three rectangles
        // You need to adjust these coordinates based on your screen resolution
        Rect rect1 = new Rect(230, 270, 100, 100);
        Rect rect2 = new Rect(440, 230, 100, 100);
        Rect rect3 = new Rect(500, 230, 100, 100);

        // Draw rectangles on the output
        drawRectangle(maskedInputMat, rect1, new Scalar(255, 0, 0)); // Blue
        drawRectangle(maskedInputMat, rect2, new Scalar(0, 255, 0)); // Green
        drawRectangle(maskedInputMat, rect3, new Scalar(0, 0, 255)); // Green

        drawRectangle(frame, rect1, new Scalar(255, 0, 0)); // Blue
        drawRectangle(frame, rect2, new Scalar(0, 255, 0)); // Green
        drawRectangle(frame, rect3, new Scalar(0, 0, 255)); // Green



        // Calculate the amount of red in each rectangle
        Mat r1 = maskedInputMat.submat(rect1);
        Mat r2 = maskedInputMat.submat(rect2);
        Mat r3 = maskedInputMat.submat(rect3);
        redAmount1 = calculateRedAmount(r1);
        redAmount2 = calculateRedAmount(r2);
        redAmount3 = calculateRedAmount(r3);
        r1.release();
        r2.release();
        r3.release();


        if (redAmount1 > redThreshold) {
            this.placementPosition = PlacementPosition.LEFT;
        } else if (redAmount2 > redThreshold) {
            this.placementPosition = PlacementPosition.CENTER;
        } else {
            this.placementPosition = PlacementPosition.RIGHT;
        }


        // Output the red amounts to the console (you can modify this part)
        return maskedInputMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {


        /*
        Paint myPaint = new Paint();
        myPaint.setColor(Color.rgb(0, 0, 0));
        myPaint.setStrokeWidth(1);

        // canvas.drawRect(100, 200, 200, 300, myPaint);
        // canvas.drawRect(350, 190, 450, 300, myPaint);
        canvas.drawRect((int) (100.0/640 * 1000), (int) (230.0/480 * 1000), (int) (200.0/640 * 1000), (int) (330.0/480 * 1000), myPaint);
        canvas.drawRect((int) (350.0/640 * 1000), (int) (230.0/480 * 1000), (int) (450.0/640 * 1000), (int) (330.0/480 * 1000), myPaint);


         */

    }


    // Helper method to calculate the amount of red in a given Mat using countNonZero
    private double calculateRedAmount(Mat mat) {
        Mat binary = new Mat();
        Imgproc.cvtColor(mat, binary, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(binary, binary, 1, 255, Imgproc.THRESH_BINARY);

        int nonZeroCount = Core.countNonZero(binary);
        binary.release();

        return nonZeroCount;
    }
    public double getRedAmount1() {
        return redAmount1;
    }

    public double getRedAmount2() {
        return redAmount2;
    }

    public double getRedAmount3() {
        return redAmount3;
    }
    private void drawRectangle(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect.tl(), rect.br(), color, 2);
    }

    public PlacementPosition getPlacementPosition() {
        return this.placementPosition;
    }

}

package org.firstinspires.ftc.teamcode.vision.simulatortests;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectionRedFar extends OpenCvPipeline {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    public Scalar lower = new Scalar(0, 151.0, 86);
    public Scalar upper = new Scalar(240, 255, 160);

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    double redAmount1 = 0;
    double redAmount2 = 0;

    double redAmount3 = 0;
    private final double redThreshold = 2500;
    private volatile PlacementPosition placementPosition = PlacementPosition.CENTER;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        // Define the coordinates of three rectangles
        // You need to adjust these coordinates based on your screen resolution
        Rect rect1 = new Rect(220, 240, 100, 100);
        Rect rect2 = new Rect(420, 240, 100, 100);
        Rect rect3 = new Rect(500, 200, 100, 100);

        // Draw rectangles on the output
        drawRectangle(maskedInputMat, rect1, new Scalar(255, 0, 0)); // Blue
        drawRectangle(maskedInputMat, rect2, new Scalar(0, 255, 0)); // Green
        drawRectangle(maskedInputMat, rect3, new Scalar(0, 0, 255)); // Green



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
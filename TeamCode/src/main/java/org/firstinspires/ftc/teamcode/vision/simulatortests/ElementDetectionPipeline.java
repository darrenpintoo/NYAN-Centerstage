package org.firstinspires.ftc.teamcode.vision.simulatortests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementDetectionPipeline extends OpenCvPipeline {

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public static Point TOP_LEFT_ANCHOR_POINT1 = new Point(145, 168);
    public static Point TOP_LEFT_ANCHOR_POINT2 = new Point(245, 168);
    public static Point TOP_LEFT_ANCHOR_POINT3 = new Point(345, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    public static Scalar lowerRed = new Scalar(0, 0, 0);
    public static Scalar upperRed = new Scalar(255, 255, 255);


    // Color definitions

    private Mat redMatrix1 = new Mat();
    private Mat redMatrix2 = new Mat();
    private Mat redMatrix3 = new Mat();

    // Anchor point definitions
    Point point1A = new Point(TOP_LEFT_ANCHOR_POINT1.x, TOP_LEFT_ANCHOR_POINT1.y);
    Point point1B = new Point(TOP_LEFT_ANCHOR_POINT1.x + REGION_WIDTH, TOP_LEFT_ANCHOR_POINT1.y + REGION_HEIGHT);
    Point point2A = new Point(TOP_LEFT_ANCHOR_POINT2.x, TOP_LEFT_ANCHOR_POINT2.y);
    Point point2B = new Point(TOP_LEFT_ANCHOR_POINT2.x + REGION_WIDTH, TOP_LEFT_ANCHOR_POINT2.y + REGION_HEIGHT);
    Point point3A = new Point(TOP_LEFT_ANCHOR_POINT3.x, TOP_LEFT_ANCHOR_POINT3.y);
    Point point3B = new Point(TOP_LEFT_ANCHOR_POINT3.x + REGION_WIDTH, TOP_LEFT_ANCHOR_POINT3.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private int stageNum = 0;

    private enum Stage
    {
        RED,
        INPUT
    }

    Stage[] stages = Stage.values();

    Telemetry t;

    public ElementDetectionPipeline(Telemetry t) {
        this.t = t;
    }
    public ElementDetectionPipeline() {}

    @Override
    public void onViewportTapped()
    {

        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Memory cleanup
        redMatrix1.release();
        redMatrix2.release();
        redMatrix3.release();

        // Noise reduction
        Imgproc.cvtColor(input, redMatrix1, Imgproc.COLOR_RGB2YCrCb);

        // Imgproc.blur(blurredMatrix, blurredMatrix, new Size(5, 5));
        // blurredMatrix = blurredMatrix.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        // Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        // Imgproc.morphologyEx(blurredMatrix, blurredMatrix, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(input, lowerRed, upperRed, redMatrix1);
        Core.inRange(input, lowerRed, upperRed, redMatrix1);
        Core.inRange(input, lowerRed, upperRed, redMatrix1);

        // Core.inRange(blurredMatrix, lowerYellow, upperYellow, yellowMatrix);
        // Core.inRange(blurredMatrix, lowerGreen, upperGreen, greenMatrix);

        // Gets color specific values
        double redPercent1 = Core.countNonZero(redMatrix1);
        double redPercent2 = Core.countNonZero(redMatrix2);
        double redPercent3 = Core.countNonZero(redMatrix3);


        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(redPercent1, Math.max(redPercent2, redPercent3));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
/*        if (maxPercent == redPercent1) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
            Imgproc.rectangle(
                    yellowMatrix,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == greenPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
            Imgproc.rectangle(
                    greenMatrix,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
        } else if (maxPercent == magentaPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
            Imgproc.rectangle(
                    magentaMatrix,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }*/


        if (this.t != null) {
            t.addData("Maximum Percent: ", maxPercent);
            t.update();
        }

        switch (stages[stageNum]) {
            case RED:
                return redMatrix1;
            case INPUT:
                return input;
        }

        return input;
    }

}
package org.firstinspires.ftc.teamcode.vision.simulatortests;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class PreloadDetectionPipeline implements VisionProcessor {


    int targetAprilTagID = 1;

    public double leftAverage;
    public double rightAverage;
    Rect leftInclusionZone;
    Rect rightInclusionZone;

    private final Paint borderPaint = new Paint();


    public Scalar lower = new Scalar(27, 10, 10);
    public Scalar upper = new Scalar(90, 255, 255);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        borderPaint.setColor(Color.MAGENTA);
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(2);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        List<AprilTagDetection> currentDetections = RobotEx.getInstance().camera.getDetections();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == targetAprilTagID) {
                        System.out.println("Tag FOUND");
                        int leftX = Integer.MAX_VALUE;
                        int rightX = Integer.MIN_VALUE;
                        int topY = Integer.MIN_VALUE;
                        int bottomY = Integer.MAX_VALUE;

                        for (Point point : detection.corners) {
                            if (point.x < leftX) leftX = (int) point.x;
                            if (point.x > rightX) rightX = (int) point.x;
                            if (point.y > topY) topY = (int) point.y;
                            if (point.y < bottomY) bottomY = (int) point.y;
                        }

                        int tagCenterX = (int) detection.center.x;
                        int tagCenterY = (int) detection.center.y;

                        int tagWidth = rightX - leftX;
                        int tagHeight = topY - bottomY;

                        int inclusionZoneWidth = (int) (tagWidth * 1.5);
                        int inclusionZoneHeight = (int) (tagHeight * 1.5);

                        int exclusionZoneWidth = (int) (tagWidth * 0.28);
                        int exclusionZoneHeight = (int) (tagHeight * 0.28);

                        leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
                        rightInclusionZone = new Rect(tagCenterX, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 7);
                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 7);


                        Mat leftRect = frame.submat(leftInclusionZone);
                        Mat rightRect = frame.submat(rightInclusionZone);

                        Imgproc.cvtColor(leftRect, leftRect, Imgproc.COLOR_RGB2HSV);
                        Imgproc.cvtColor(rightRect, rightRect, Imgproc.COLOR_RGB2HSV);

                        Core.inRange(leftRect, lower, upper, leftRect);
                        Core.inRange(rightRect, lower, upper, rightRect);

                        double leftZoneAverage = calculateYellowAmount(leftRect);
                        double rightZoneAverage = calculateYellowAmount(rightRect);


                        leftAverage = leftZoneAverage;
                        rightAverage = rightZoneAverage;

                        leftRect.release();
                        rightRect.release();
                    }
                }
            }
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        if (leftInclusionZone != null) {
            RectF stackRectL = new RectF(leftInclusionZone.x * scaleBmpPxToCanvasPx, leftInclusionZone.y * scaleBmpPxToCanvasPx, (leftInclusionZone.x + leftInclusionZone.width) * scaleBmpPxToCanvasPx, (leftInclusionZone.y + leftInclusionZone.height) * scaleBmpPxToCanvasPx);
            RectF stackRectR = new RectF(rightInclusionZone.x * scaleBmpPxToCanvasPx, rightInclusionZone.y * scaleBmpPxToCanvasPx, (rightInclusionZone.x + rightInclusionZone.width) * scaleBmpPxToCanvasPx, (rightInclusionZone.y + rightInclusionZone.height) * scaleBmpPxToCanvasPx);

            canvas.drawRect(stackRectL, borderPaint);
            canvas.drawRect(stackRectR, borderPaint);


        }
    }

    private double calculateYellowAmount(Mat mat) {
        Mat binary = new Mat();

        Imgproc.threshold(binary, binary, 1, 255, Imgproc.THRESH_BINARY);

        int nonZeroCount = Core.countNonZero(binary);
        binary.release();

        return nonZeroCount;
    }
    public void setTargetAprilTagID(int target) {
        targetAprilTagID = target;
    }


}

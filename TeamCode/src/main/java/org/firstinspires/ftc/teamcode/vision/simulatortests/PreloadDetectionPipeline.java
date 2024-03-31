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


    public int targetAprilTagID = 3;

    public double leftAverage;
    public double rightAverage;
    Rect leftInclusionZone;
    Rect rightInclusionZone;

    public Scalar lower = new Scalar(27, 10, 10);
    public Scalar upper = new Scalar(90, 255, 255);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

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

                        leftAverage = meanColor(frame, leftInclusionZone);
                        rightAverage = meanColor(frame, rightInclusionZone);

                    }
                }
            }
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public double meanColor(Mat frame, Rect inclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y + inclusionRect.height / 2 - 3; y < inclusionRect.y + inclusionRect.height / 2 + 3; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
                    continue;
                }


                double[] data = frame.get(y, x);
                if (data != null && data.length > 0) {
                    sum += data[0];
                    count++;
                }
            }
        }

        return count > 0 ? (double) sum / count: 0;
    }

    public double meanColorLinear(Mat frame, Rect inclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        int sum = 0;
        int count = 0;

        int y = (int) Math.floor(inclusionRect.y / 2.0);
        int x = (int) Math.floor(inclusionRect.x / 2.0);

        for (int i = inclusionRect.x; i < inclusionRect.x + inclusionRect.width; i++) {
            if (i < 0 || i >= frame.cols() || y < 0 || y >= frame.rows()) {
                continue;
            }


            double[] data = frame.get(y, i);
            if (data != null && data.length > 0) {
                sum += data[0];
                count++;
            }
        }

        for (int i = inclusionRect.y; i < inclusionRect.y + inclusionRect.height; i++) {
            if (x < 0 || x >= frame.cols() || i < 0 || i >= frame.rows()) {
                continue;
            }


            double[] data = frame.get(i, x);
            if (data != null && data.length > 0) {
                sum += data[0];
                count++;
            }
        }

        return count > 0 ? (double) sum / count: 0;

    }
    public void setTargetAprilTagID(int target) {
        targetAprilTagID = target;
    }

}

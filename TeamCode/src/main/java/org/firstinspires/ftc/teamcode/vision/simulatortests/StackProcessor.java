package org.firstinspires.ftc.teamcode.vision.simulatortests;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class StackProcessor implements VisionProcessor {
    public static final double STACK_WIDTH = 3.25;
    public double focalLength = 821.993; // 640x480
    public double CAMERA_HEIGHT = 0;
    public double CAMERA_WIDTH = 0;
    private final Paint borderPaint = new Paint();
    private final Paint textPaint = new Paint();
    private Mat processingMat = new Mat();
    private Rect stackRect = null;
    private Rect tapeRect = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        CAMERA_HEIGHT = height;
        CAMERA_WIDTH = width;

        borderPaint.setColor(Color.MAGENTA);
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(2);

        textPaint.setColor(Color.MAGENTA);
        textPaint.setTextSize(20);

        focalLength = calibration.focalLengthY;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();

        Imgproc.cvtColor(frame, processingMat, Imgproc.COLOR_BGR2HLS);

        Core.inRange(processingMat, new Scalar(0, 180, 0), new Scalar(180, 255, 255), processingMat);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(processingMat, processingMat, kernel);
        Imgproc.dilate(processingMat, processingMat, kernel);

        Imgproc.findContours(processingMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contoursList) {
            Rect boundingRect = Imgproc.boundingRect(new MatOfPoint(contour.toArray()));
            if (boundingRect.width < 75) {
                continue;
            }
            if (boundingRect.y < 80) continue;
            if (boundingRect.x > 300) continue;
            for (int y = boundingRect.y + boundingRect.height / 2; y < boundingRect.y + boundingRect.height; y++) {
                int count = 0;
                int black = 0;

                for (int x = boundingRect.x; x < boundingRect.x + boundingRect.width; x++) {
                    double[] pixelValue = processingMat.get(y, x);
                    if (pixelValue[0] == 0) {
                        black++;
                    }
                    count++;
                }

                if ((double)black/count > 0.6) {
                    stackRect = new Rect(
                            boundingRect.x,
                            y,
                            boundingRect.width,
                            boundingRect.y + boundingRect.height - y
                    );

                    tapeRect = new Rect(
                            boundingRect.x,
                            boundingRect.y,
                            boundingRect.width,
                            y - boundingRect.y
                    );
                    break;
                } else if (stackRect != null || tapeRect != null) {
                    stackRect = null;
                    tapeRect = null;
                }
            }
        }


        processingMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (stackRect != null) {
            RectF stackRectF = new RectF(stackRect.x * scaleBmpPxToCanvasPx, stackRect.y * scaleBmpPxToCanvasPx, (stackRect.x + stackRect.width) * scaleBmpPxToCanvasPx, (stackRect.y + stackRect.height) * scaleBmpPxToCanvasPx);
            borderPaint.setColor(Color.MAGENTA);
            canvas.drawRect(stackRectF, borderPaint);
        }

        if (tapeRect != null) {
            RectF tapeRectF = new RectF(tapeRect.x * scaleBmpPxToCanvasPx, tapeRect.y * scaleBmpPxToCanvasPx, (tapeRect.x + tapeRect.width) * scaleBmpPxToCanvasPx, (tapeRect.y + tapeRect.height) * scaleBmpPxToCanvasPx);
            borderPaint.setColor(Color.GREEN);
            canvas.drawRect(tapeRectF, borderPaint);
        }
    }

    public Double getStrafeError() {
        if (stackRect == null) {
            return null;
        }

        double dZ = (STACK_WIDTH * focalLength) / stackRect.width;
        double dY = (dZ / focalLength) * (stackRect.x - 308);

        return dY;
    }

//    public Vector2d getDistanceError(double robotHeading) {
//        if (stackRect == null) {
//            return null;
//        }
//
//        double rotatedHeading = -robotHeading;
//
//        double dZ = (STACK_WIDTH * FOCAL_LENGTH) / stackRect.width;
//        double dX = Math.cos(robotHeading) * dZ - FRONT_CAMERA_OFFSET_Y;
////        double dY = Math.sin(robotHeading) * dZ - FRONT_CAMERA_OFFSET_X;
//        double dY = (dZ / FOCAL_LENGTH) * (stackRect.x - 320);
//
////        double rotatedX = dX * Math.cos(rotatedHeading) + dY * Math.sin(rotatedHeading);
////        double rotatedY = dX * -Math.sin(rotatedHeading) + dY * Math.cos(rotatedHeading);
//
//        return new Vector2d(dX, dY);
//    }
}

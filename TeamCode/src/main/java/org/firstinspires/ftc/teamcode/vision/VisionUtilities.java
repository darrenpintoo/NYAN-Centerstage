package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class VisionUtilities {

    private static final Paint borderPaint = new Paint();


    public static void drawRectangle(Canvas canvas, Rect rect, float ratio) {
        RectF rectF = new RectF(rect.x * ratio, rect.y * ratio, (rect.x + rect.width) * ratio, (rect.y + rect.height) * ratio);

        borderPaint.setColor(Color.MAGENTA);
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(2);

        canvas.drawRect(rectF, borderPaint);

    }
    public static void drawRectangle(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect.tl(), rect.br(), color, 2);
    }

    public static double calculateAmount(Mat mat) {
        Mat binary = new Mat();
        Imgproc.cvtColor(mat, binary, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(binary, binary, 1, 255, Imgproc.THRESH_BINARY);

        int nonZeroCount = Core.countNonZero(binary);
        binary.release();

        return nonZeroCount;
    }
}

package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utilities.robot.Alliance;
import org.firstinspires.ftc.teamcode.utilities.robot.Globals;
import org.firstinspires.ftc.teamcode.utilities.robot.Side;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class PropPipeline implements VisionProcessor {
    private static final boolean DEBUG = false;
    public static int redLeftX = (int) (80);
    public static int redLeftY = (int) (200);
    public static int redCenterX = (int) (375);
    public static int redCenterY = (int) (200);
    public static int blueLeftX = (int) (200);
    public static int blueLeftY = (int) (100);
    public static int blueCenterX = (int) (300);
    public static int blueCenterY = (int) (100);
    public static int leftWidth = (int) (50);
    public static int leftHeight = (int) (50);
    public static int centerWidth = (int) (50);
    public static int centerHeight = (int) (50);
    public static double BLUE_TRESHOLD = 70;
    public static double RED_TRESHOLD = 100;
    public double leftColor = 0.0;
    public double centerColor = 0.0;
    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);

    public Scalar redLower = new Scalar(0, 0, 0);
    public Scalar redUpper = new Scalar(20, 255, 255);
    public Scalar blueLower = new Scalar(150, 0, 0);
    public Scalar blueUpper = new Scalar(230, 255, 255);
    Telemetry telemetry;

    Rect leftZoneArea;
    Rect centerZoneArea;
    private volatile PlacementPosition location = PlacementPosition.LEFT;

//    Location ALLIANCE = Location.RED;

    public PropPipeline() {
        this(null);
    }

    public PropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (frame == null || frame.empty()) {
            return null;
        }

        Mat workingMat = new Mat();
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_RGB2HSV);

        Scalar lower;
        Scalar upper;

        if (Globals.ALLIANCE == Alliance.RED) {
            lower = redLower;
            upper = redUpper;
        } else {
            lower = blueLower;
            upper = blueUpper;
        }

        Core.inRange(workingMat, lower, upper, workingMat);

        if (Globals.ALLIANCE == Alliance.RED && Globals.SIDE == Side.FAR || Globals.ALLIANCE == Alliance.BLUE && Globals.SIDE == Side.CLOSE) {
            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
        } else {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, centerWidth, centerHeight);
        }

        Mat leftZone = workingMat.submat(leftZoneArea);
        Mat centerZone = workingMat.submat(centerZoneArea);


        if (DEBUG) {
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);
        }

        leftColor = Core.countNonZero(leftZone);
        centerColor = Core.countNonZero(centerZone);

        if (telemetry != null) {
            telemetry.addData("leftColor", leftColor);
            telemetry.addData("centerColor", centerColor);
            telemetry.addData("analysis", location.toString());
            telemetry.update();
        }

        if (leftColor > BLUE_TRESHOLD) {
            location = PlacementPosition.LEFT;
        } else if (centerColor > BLUE_TRESHOLD) {
            location = PlacementPosition.CENTER;
        } else {
            location = PlacementPosition.RIGHT;
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public PlacementPosition getLocation() {
        return this.location;
    }

    public double getLeftColor() {
        return leftColor;
    }

    public double getCenterColor() {
        return centerColor;
    }

}
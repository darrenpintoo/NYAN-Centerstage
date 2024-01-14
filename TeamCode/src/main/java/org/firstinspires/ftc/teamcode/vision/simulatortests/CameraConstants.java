package org.firstinspires.ftc.teamcode.vision.simulatortests;

import org.opencv.core.Mat;

public class CameraConstants {


    private final static double[][] CAMERA_MATRIX_ARRAY = new double[][]{
            new double[] {1.02345122e+03, 0D, 5.92369453e+02},
            new double[] {0D, 1.01729018e+03, 3.75351779e+02},
            new double[] {0D, 0D, 1D}
    };

    public final static int WIDTH = 1280;
    public final static int HEIGHT = 720;

    public final static double fx = CAMERA_MATRIX_ARRAY[0][0];
    public final static double fy = CAMERA_MATRIX_ARRAY[1][1];

    public final static double cx = CAMERA_MATRIX_ARRAY[0][2];
    public final static double cy = CAMERA_MATRIX_ARRAY[1][2];

    public final static double fovX = 2 * Math.atan2(WIDTH, 2 * fx);
    public final static double fovY = 2 * Math.atan2(HEIGHT, 2 * fy);

    public final static double fovXDeg = Math.toDegrees(2 * Math.atan2(WIDTH, 2 * fx));
    public final static double fovYDeg = Math.toDegrees(2 * Math.atan2(HEIGHT, 2 * fy));

    public final static long MS_TO_PROCESS_FRAME = 2;

}

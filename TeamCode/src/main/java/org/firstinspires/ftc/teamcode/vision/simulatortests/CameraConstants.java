package org.firstinspires.ftc.teamcode.vision.simulatortests;

import org.opencv.core.Mat;

public class CameraConstants {

    public static class FrontCamera {
        private final static double[][] CAMERA_MATRIX_ARRAY = new double[][]{
                new double[]{633.978, 0D, 305.785},
                new double[]{0D, 633.978, 248.325},
                new double[]{0D, 0D, 1D}
        };

        public final static int WIDTH = 640;
        public final static int HEIGHT = 480;

        public final static double fx = CAMERA_MATRIX_ARRAY[0][0];
        public final static double fy = CAMERA_MATRIX_ARRAY[1][1];

        public final static double cx = CAMERA_MATRIX_ARRAY[0][2];
        public final static double cy = CAMERA_MATRIX_ARRAY[1][2];

        public final static double fovX = 2 * Math.atan2(WIDTH, 2 * fx);
        public final static double fovY = 2 * Math.atan2(HEIGHT, 2 * fy);

        public final static double fovXDeg = Math.toDegrees(2 * Math.atan2(WIDTH, 2 * fx));
        public final static double fovYDeg = Math.toDegrees(2 * Math.atan2(HEIGHT, 2 * fy));

        public final static long MS_TO_PROCESS_FRAME = 2;
        public final static long exposure = 100;// 25;
        public final static int gain = 100;
    }


    public static class BackCamera {
        private final static double[][] CAMERA_MATRIX_ARRAY = new double[][]{
                new double[]{640.861, 0D, 313.689},
                new double[]{0D, 640.861, 255.993},
                new double[]{0D, 0D, 1D}
        };
                /*new double[][]{
                new double[]{628.402, 0D, 309.25},
                new double[]{0D, 628.402, 242.842},
                new double[]{0D, 0D, 1D}
        };

                 */

        public final static int WIDTH = 640;
        public final static int HEIGHT = 480;

        public final static double fx = CAMERA_MATRIX_ARRAY[0][0];
        public final static double fy = CAMERA_MATRIX_ARRAY[1][1];

        public final static double cx = CAMERA_MATRIX_ARRAY[0][2];
        public final static double cy = CAMERA_MATRIX_ARRAY[1][2];

        public final static double fovX = 2 * Math.atan2(WIDTH, 2 * fx);
        public final static double fovY = 2 * Math.atan2(HEIGHT, 2 * fy);

        public final static double fovXDeg = Math.toDegrees(2 * Math.atan2(WIDTH, 2 * fx));
        public final static double fovYDeg = Math.toDegrees(2 * Math.atan2(HEIGHT, 2 * fy));

        public final static long MS_TO_PROCESS_FRAME = 2;

        public final static long exposure = 10;
        public final static int gain = 100;
    }

    private final static double[][] CAMERA_MATRIX_ARRAY = new double[][]{
            new double[]{1.02345122e+03, 0D, 5.92369453e+02},
            new double[]{0D, 1.01729018e+03, 3.75351779e+02},
            new double[]{0D, 0D, 1D}
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

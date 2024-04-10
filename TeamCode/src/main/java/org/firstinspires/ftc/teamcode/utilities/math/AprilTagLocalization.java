package org.firstinspires.ftc.teamcode.utilities.math;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagLocalization {
    public static int getBackboardIdFromDetection(PlacementPosition detectionResult) {
        if (true) {
            switch (detectionResult) {
                case LEFT:
                    return 1;
                case CENTER:
                    return 2;
                default:
                    return 3;
            }
        } else {
            switch (detectionResult) {
                case LEFT:
                    return 4;
                case CENTER:
                    return 5;
                default:
                    return 6;
            }
        }
    }

    public static Pose getTagPosition(AprilTagDetection detection) {
        switch (detection.id) {
            case 1:
                return new Pose(-41.41, -60, 0);
            case 2:
                return new Pose(-35.41, -60, 0);
            case 3:
                return new Pose(-29.41, -60, 0);
            case 4:
                return new Pose(29.41, -60, 0);
            case 5:
                return new Pose(35.41, -60, 0);
            case 6:
                return new Pose(41.41, -60, 0);
            case 7:
                return new Pose(40.625, 70.25, 0);
            case 8:
                return new Pose(35.125, 70.25, 0);
            case 9:
                return new Pose(-35.125, 70.25, 0);
            case 10:
                return new Pose(-40.625, 70.25, 0);
            default:
                return null;
        }
    }

    public static Pose getTagPosition(int detectionId) {
        switch (detectionId) {
            case 1:
                return new Pose(-41.41, -60, 0);
            case 2:
                return new Pose(-35.41, -60, 0);
            case 3:
                return new Pose(-29.41, -60, 0);
            case 4:
                return new Pose(29.41, -60, 0);
            case 5:
                return new Pose(35.41, -60, 0);
            case 6:
                return new Pose(41.41, -60, 0);
            case 7:
                return new Pose(40.625, 70.25, 0);
            case 8:
                return new Pose(35.125, 70.25, 0);
            case 9:
                return new Pose(-35.125, -70.25, 0);
            case 10:
                return new Pose(-40.625, -70.25, 0);
            default:
                return new Pose();
        }
    }

    public static Pose getRobotPositionFromBackTag(AprilTagDetection detection, double robotHeading, Pose offset) {
        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;

        double rotatedHeading = -robotHeading;

        double x2 = x * Math.cos(rotatedHeading) + y * Math.sin(rotatedHeading);
        double y2 = x * -Math.sin(rotatedHeading) + y * Math.cos(rotatedHeading);

        double x3 = offset.getX() * Math.cos(rotatedHeading) + offset.getY() * Math.sin(rotatedHeading);
        double y3 = offset.getX() * -Math.sin(rotatedHeading) + offset.getY() * Math.cos(rotatedHeading);

        double absX;
        double absY;

        Pose tagpose = getTagPosition(detection);
        tagpose.add(new Pose(-x2, y2, 0));
        tagpose.add(new Pose(-y3, -x3, 0));

        return tagpose;
    }

    public static Pose getRobotPositionFromFrontTag(AprilTagDetection detection, double robotHeading, Pose offset) {
        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;

        double rotatedHeading = -robotHeading;

        double x2 = x * Math.cos(rotatedHeading) + y * Math.sin(rotatedHeading);
        double y2 = x * -Math.sin(rotatedHeading) + y * Math.cos(rotatedHeading);

        double x3 = offset.getX() * Math.cos(rotatedHeading) + offset.getY() * Math.sin(rotatedHeading);
        double y3 = offset.getX() * -Math.sin(rotatedHeading) + offset.getY() * Math.cos(rotatedHeading);

        double absX;
        double absY;

        Pose tagpose = getTagPosition(detection);
        tagpose.add(new Pose(x2, -y2, 0));
        // tagpose.add(new Pose(-y3, -x3, 0));

        return tagpose;
    }

    public static Pose getRobotPositionFromBackTag(AprilTagDetection detection, double robotHeading, double offsetX, double offsetY) {
        double x = detection.ftcPose.x + offsetX;
        double y = detection.ftcPose.y + offsetY;

        double rotatedHeading = -robotHeading;

        double x2 = x * Math.cos(rotatedHeading) + y * Math.sin(rotatedHeading);
        double y2 = x * -Math.sin(rotatedHeading) + y * Math.cos(rotatedHeading);

        double absX;
        double absY;

        VectorF tagpose = detection.metadata.fieldPosition;
        if (detection.metadata.id <= 6) {
            absX = tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;
        } else {
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2;
        }

        return new Pose(absX, absY, robotHeading);
    }
}
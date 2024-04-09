package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import org.firstinspires.ftc.teamcode.utilities.math.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;

public class MovementUtils {

    public enum BackdropPosition {
        LEFT,
        RIGHT,
        AUTO
    }

    public enum SpikemarkPosition{
        LEFT,
        RIGHT
    }

    public static void preloadCloseBlue(RobotEx robot, PIDDrive drive, PlacementPosition position) {
        Pose placementPose = AprilTagLocalization.getTagPosition(position.getPosition());

        drive.gotoPoint(placementPose.addGet(new Pose(3, 0, 0)), 0);
        robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
        drive.gotoPoint(placementPose.addGet(new Pose(0, 12, 0)), -0.25);
        drive.gotoPoint(placementPose.addGet(new Pose(0, 10, 0)), 0);
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.25);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        drive.gotoPoint(new Pose(-25, -24, 0), -0.1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        robot.intake.reset();
        robot.pause(0.1);
        drive.gotoPoint(new Pose(-13, -30, 0), -0.25);
    }

    public static double getOffsetFromBackdropPlacement(RobotEx robot) {
        switch (robot.camera.backdropPosition) {
            case LEFT:
                return -2;
            case RIGHT:
                return 2;
        }
        return 0;
    }
}

package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

public class PIDDrive {
    GeneralPIDController xController = new GeneralPIDController(0.3, 0, 0, 0);
    GeneralPIDController yController = new GeneralPIDController(0.3, 0, 0, 0);
    GeneralPIDController headingController = new GeneralPIDController(1.5, 0, 0, 0);

    RobotEx robot;

    Telemetry telemetry;
    public PIDDrive(RobotEx robot, Telemetry t) {
        this.robot = robot;
        this.telemetry = t;
    }
    public static Pose threshold = new Pose(1, 1, 0.05);
    public static double thresholdTime = 1;
    public void gotoPoint(Pose point) {
        Pose error = new Pose(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        boolean inPosition = false;
        ElapsedTime inPositionTime = new ElapsedTime();
        while (true) {

            Pose currentPose = robot.localizer.getPose();
            error = new Pose(
                    point.getX() - currentPose.getX(),
                    point.getY() - currentPose.getY(),
                    point.getHeading() - currentPose.getHeading()
            );

            robot.drivetrain.fieldCentricDriveFromGamepad(
                    -MathUtils.clamp(xController.getOutputFromError(
                            error.getX()
                    ), -0.6, 0.6),
                    MathUtils.clamp(yController.getOutputFromError(
                            error.getY()
                    ), -0.6, 0.6),
                    -MathUtils.clamp(headingController.getOutputFromError(
                             error.getHeading()
                    ), -0.6, 0.6)
            );


            telemetry.addData("X: ", error.getX());
            telemetry.addData("Y: ", error.getY());
            telemetry.addData("Heading: ", error.getHeading());
            telemetry.update();
            robot.update();

            if (Math.abs(error.getX()) < threshold.getX() & Math.abs(error.getY()) < threshold.getY() & Math.abs(error.getHeading()) < threshold.getHeading()) {
                if (inPosition) {
                    if (inPositionTime.seconds() > thresholdTime) {
                        break;
                    }
                } else {
                    inPosition = true;
                    inPositionTime.reset();
                }
            } else {
                inPosition = false;
            }

        }

        robot.drivetrain.fieldCentricDriveFromGamepad(
                0,
                0,
                0
        );

        robot.update();

    }
}

package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import static org.firstinspires.ftc.teamcode.utilities.robot.movement.MotionProfileLocalizerLineDrive.kStaticTurn;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.MotionProfiledMotion;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class PIDDrive {
   //  GeneralPIDController xController = new GeneralPIDController(0.3, 0, 0, 0);
    // GeneralPIDController yController = new GeneralPIDController(0.3, 0, 0, 0);

    GeneralPIDController xController = new GeneralPIDController(0.2,0, 0, 0);
    GeneralPIDController yController = new GeneralPIDController(0.2, 0, 0, 0);
    GeneralPIDController headingController = new GeneralPIDController(1.5, 0, 0, 0);

    public static double vMax = 55;
    public static double aMax = 55;

    public static double kA = 0.003;
    public static double kV = 1/vMax;

    RobotEx robot;

    Telemetry telemetry;
    LinearOpMode opmode;

    public PIDDrive(RobotEx robot, LinearOpMode opmode, Telemetry t) {
        this.robot = robot;
        this.telemetry = t;
        this.opmode = opmode;
    }
    public static Pose threshold = new Pose(1, 1, Math.toRadians(2));
    public static double thresholdTime = 0.25;
    public void gotoPoint(Pose point) {
        this.gotoPoint(
                point,
                new MovementConstants(vMax, aMax, DriveConstants.MAX_CORRECTION_TIME)
        );
    }

    public void gotoPoint(Pose point, double correctionTime) {
        this.gotoPoint(
                point,
                new MovementConstants(vMax, aMax, correctionTime)
        );
    }
    public void gotoPoint(Pose point, MovementConstants constants) {
        Pose error = new Pose(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        Pose currentPose = robot.localizer.getPose();
        Pose currentVelocity = robot.localizer.getVelocity();

        Pose startPosition = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        error = new Pose(
                point.getX() - currentPose.getX(),
                point.getY() - currentPose.getY(),
                point.getHeading() - currentPose.getHeading()
        );

        double displacement = Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY());

        MotionProfile motion = new MotionProfile(
                0, displacement, constants.velocityMax, constants.accelerationMax
        );

        double angle = Math.atan2(error.getY(), error.getX());
        double sineTerm = Math.sin(angle);
        double cosineTerm = Math.cos(angle);

        double duration = motion.getDuration();
        double currentProfileTime = 0;

        boolean inPosition = false;

        ElapsedTime inPositionTime = new ElapsedTime();
        ElapsedTime profileTime = new ElapsedTime();

        while (!robot.stopRequested) {

            currentPose = robot.localizer.getPose();
            currentVelocity = robot.localizer.getVelocity();
            currentProfileTime = profileTime.seconds();

            double targetDisplacement = motion.getPositionFromTime(currentProfileTime);
            double xTarget = cosineTerm * targetDisplacement + startPosition.getX();
            double yTarget = sineTerm * targetDisplacement + startPosition.getY();
            // double headingTarget = MathHelper.lerp(startPosition.getHeading(), point.getHeading(), Math.min(currentProfileTime, duration) / duration);

            error = new Pose(
                    xTarget - currentPose.getX(),
                    yTarget - currentPose.getY(),
                    point.getHeading() - currentPose.getHeading()
            );

            double feedforward = motion.getAccelerationFromTime(currentProfileTime) * kA + motion.getVelocityFromTime(currentProfileTime) * kV;

            double feedforwardX = feedforward * cosineTerm;
            double feedforwardY = feedforward * sineTerm;

            double feedbackX = xController.getOutputFromError(error.getX());
            double feedbackY = yController.getOutputFromError(error.getY());

            robot.drivetrain.fieldCentricDriveFromGamepad(
                    feedbackX + feedforwardX,
                    feedbackY + feedforwardY,
                    -MathUtils.clamp(headingController.getOutputFromError(
                            error.getHeading()
                    ), -0.5, 0.5)
            );


            telemetry.addData("Profile time: ", currentProfileTime);
            telemetry.addData("Motion time: ", duration);

            robot.update();
            error.map(Math::abs);
            currentVelocity = new Pose(currentVelocity).map(Math::abs);

            if (error.lessThan(threshold) && currentProfileTime > duration) {
                if (inPosition) {
                    if (inPositionTime.seconds() >= thresholdTime || currentVelocity.lessThan(threshold)) {
                        break;
                    }
                } else {
                    inPosition = true;
                    inPositionTime.reset();
                }
            } else if (currentProfileTime > duration + constants.maxCorrectionTime) {
                break;
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


    public void oldGotoPoint(Pose point) {
        Pose error = new Pose(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        Pose currentPose = robot.localizer.getPose();
        error = new Pose(
                point.getX() - currentPose.getX(),
                point.getY() - currentPose.getY(),
                point.getHeading() - currentPose.getHeading()
        );

        MotionProfile motion = new MotionProfile(
                0, Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY()), vMax, aMax
        );

        boolean inPosition = false;

        ElapsedTime inPositionTime = new ElapsedTime();
        ElapsedTime profileTime = new ElapsedTime();

        while (true) {

            currentPose = robot.localizer.getPose();
            double targetDisplacement = motion.getPositionFromTime(profileTime.time());

            error = new Pose(
                    point.getX() - currentPose.getX(),
                    point.getY() - currentPose.getY(),
                    point.getHeading() - currentPose.getHeading()
            );

            robot.drivetrain.fieldCentricDriveFromGamepad(
                    MathUtils.clamp(xController.getOutputFromError(
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


    public void turnToAngle(double angle, MovementConstants constants) {
        angle = AngleHelper.normDelta(angle);

        double currentIMUPosition = robot.localizer.getPose().getHeading();
        double turnError;

        MotionProfile turnProfile = new MotionProfile(currentIMUPosition, angle, DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_VELOCITY);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime elapsedTurnTime = new ElapsedTime();

        boolean atTarget = false;
        double atTargetStartTime = -1;

        while (!atTarget && elapsedTurnTime.seconds() < turnProfile.getDuration()+constants.maxCorrectionTime) {

            double currentTargetAngle = turnProfile.getPositionFromTime(elapsedTurnTime.seconds());
            turnError = currentTargetAngle  - currentIMUPosition;

            if (Math.abs(turnError) > Math.PI) {
                if (angle < 0) {
                    angle = AngleHelper.norm(angle);
                    turnError = angle - currentIMUPosition;
                } else if (angle > 0) {
                    currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                    turnError = angle - currentIMUPosition;
                }
            }

            double output = robot.drivetrain.profiledTurningPID.getOutputFromError(
                    turnError
            );


            robot.drivetrain.robotCentricDriveFromGamepad(
                    0,
                    0,
                    -Math.min(Math.max(output, -1), 1) + Math.signum(output) * kStaticTurn
            );

            currentIMUPosition = robot.localizer.getPose().getHeading();

            if (telemetry != null) {
                telemetry.addData("Target Angle: ", currentTargetAngle);
                telemetry.addData("Turn Angle: ", turnError);
                telemetry.addData("current angle: ", currentIMUPosition);
                telemetry.update();
            }

            this.robot.update();
        }

        MotorGroup<DcMotorEx> robotDrivetrain = robot.drivetrain.getDrivetrainMotorGroup();

        DcMotor.ZeroPowerBehavior currentZeroPowerBehavior = robot.drivetrain.getZeroPowerBehavior();

        if (currentZeroPowerBehavior != DcMotor.ZeroPowerBehavior.BRAKE) {
            robotDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robotDrivetrain.setPower(0);
            robot.pause(0.1);
            robotDrivetrain.setZeroPowerBehavior(currentZeroPowerBehavior);
        }




    }

    public void turnToAngle(double angle) {
        turnToAngle(angle, new MovementConstants(0, 0, DriveConstants.MAX_CORRECTION_TIME));
    }
}

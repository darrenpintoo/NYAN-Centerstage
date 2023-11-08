package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class MotionProfileLocalizerLineDrive {
    GeneralPIDController followerPID = new GeneralPIDController(0.3, 0, 0, 0);
    GeneralPIDController laterialPID = new GeneralPIDController(0.3, 0, 0, 0);

    public static double kV = 0.01;//1 / DriveConstants.MAX_VELOCITY;
    public static double kA = 0.001;

    public static double kStaticTurn = 0.07;
    public static double kStaticMovement = 0.08;

    RobotEx robot = RobotEx.getInstance();

    InternalIMU imu = robot.internalIMU;
    Drivetrain dt = robot.drivetrain;

    Telemetry telemetry;

    LinearOpMode currentOpmode;

    ElapsedTime profileTimer = new ElapsedTime();
    ElapsedTime correctionTimer = new ElapsedTime();

    public MotionProfileLocalizerLineDrive(LinearOpMode currentOpmode) {
        this.currentOpmode = currentOpmode;
    }

    public MotionProfileLocalizerLineDrive(LinearOpMode currentOpmode, Telemetry telemetry) {
        this.currentOpmode = currentOpmode;
        this.telemetry = telemetry;
    }

    public void turnToAngle(double angle) {
        this.imu.trackAngularVelocity();

        angle = AngleHelper.normDelta(angle);

        double currentIMUPosition = this.imu.getCurrentFrameHeadingCCW();
        double currentIMUVelocity;
        double turnError;

        MotionProfile turnProfile = new MotionProfile(currentIMUPosition, angle, DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_VELOCITY);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime elapsedTurnTime = new ElapsedTime();

        boolean atTarget = false;
        double atTargetStartTime = -1;

        while (!atTarget && !this.currentOpmode.isStopRequested() && elapsedTurnTime.seconds() < turnProfile.getDuration()+DriveConstants.MAX_CORRECTION_TIME) {

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

            double output = this.dt.profiledTurningPID.getOutputFromError(
                    turnError
            );


            this.dt.robotCentricDriveFromGamepad(
                    0,
                    0,
                    Math.min(Math.max(output, -0.2), 0.2) + Math.signum(output) * kStaticTurn
            );

            currentIMUPosition = this.imu.getCurrentFrameRobotOrientation().getCCWHeading();
            currentIMUVelocity = this.imu.getCurrentFrameRobotVelocity().getTurnVelocity();

            if (telemetry != null) {
                telemetry.addData("Target Angle: ", currentTargetAngle);
                telemetry.addData("Turn Angle: ", turnError);
                telemetry.addData("current angle: ", currentIMUPosition);
                telemetry.update();
            }

            this.robot.update();
        }

        this.imu.stopAngularVelocityTracking();

        MotorGroup<DcMotorEx> robotDrivetrain = robot.drivetrain.getDrivetrainMotorGroup();

        DcMotor.ZeroPowerBehavior currentZeroPowerBehavior = robot.drivetrain.getZeroPowerBehavior();

        if (currentZeroPowerBehavior != DcMotor.ZeroPowerBehavior.BRAKE) {
            robotDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robotDrivetrain.setPower(0);
            robot.pause(0.1);
            robotDrivetrain.setZeroPowerBehavior(currentZeroPowerBehavior);
        }


    }

}
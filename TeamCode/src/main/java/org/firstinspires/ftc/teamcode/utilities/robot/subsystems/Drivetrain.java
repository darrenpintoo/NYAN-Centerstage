package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.RobotOrientation;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.EncoderDrive;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

import java.util.ArrayList;

/**
 * Robot Drivetrain
 */

@Config
public class Drivetrain implements Subsystem {

    public enum TurnDirection {
        LEFT, RIGHT
    }
    private final DcMotorEx.ZeroPowerBehavior START_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    private RobotEx robotInstance;
    private InternalIMU internalIMU;
    private TwoWheelLocalizer localizer;

    MotorGroup<DcMotorEx> drivetrainMotorGroup;
    private DcMotorEx[] drivetrainMotors;

    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;


    private boolean enableAntiTip = false;
    private boolean enableHeadingRetention = false;

    public GeneralPIDController headingPID = new GeneralPIDController(1.5, 0, 20, 0);
    public GeneralPIDController profiledTurningPID = new GeneralPIDController(1.5, 0, 40, 0);

    public GeneralPIDController translationalPID = new GeneralPIDController(1, 0, 0, 0);

    public GeneralPIDController tiltPID = new GeneralPIDController(0.3, 0, 0, 0);
    public GeneralPIDController yawPID = new GeneralPIDController(0.3, 0, 0, 0);

    private Telemetry telemetry;

    private double rightFrontPower = 0;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    private double lastRightFrontPower = 0;
    private double lastLeftFrontPower = 0;
    private double lastLeftBackPower = 0;
    private double lastRightBackPower = 0;

    private double targetTurnAngle = 0;

    private double lastX = 0;
    private double lastY = 0;
    private double lastRot = 0;



    private double weight = 1;

    public static double kP = 2;
    public static double kI = 0;
    public static double kD = 40;

    private double trackWidth = 12;
    private double wheelBase = 6.5;
    private double lateralMultiplier = -1.2;
    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        this.internalIMU = InternalIMU.getInstance();
        this.robotInstance = RobotEx.getInstance();

        this.rightFrontMotor = new CachingDcMotorEX((DcMotorEx) hardwareMap.get(DcMotor.class, "rightFrontMotor"), 1e-9);
        this.leftFrontMotor = new CachingDcMotorEX((DcMotorEx) hardwareMap.get(DcMotor.class, "leftFrontMotor"), 1e-9);
        this.leftBackMotor = new CachingDcMotorEX((DcMotorEx) hardwareMap.get(DcMotor.class, "leftBackMotor"), 1e-9);
        this.rightBackMotor = new CachingDcMotorEX((DcMotorEx) hardwareMap.get(DcMotor.class, "rightBackMotor"), 1e-9);

        // todo: figure out the directions

        this.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        this.leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);

/*        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        this.drivetrainMotors = new DcMotorEx[] {
            this.rightFrontMotor,
            this.leftFrontMotor,
            this.leftBackMotor,
            this.rightBackMotor
        };

        this.drivetrainMotorGroup = new MotorGroup<>(
                this.rightFrontMotor,
                this.leftFrontMotor,
                this.leftBackMotor,
                this.rightBackMotor
        );

        this.telemetry = telemetry;
        // this.headingPID.setTelemetry(telemetry);
    }

    public void enableAntiTip() {
        this.enableAntiTip = true;
    }

    public void disableAntiTip() {
        this.enableAntiTip = false;
    }

    public void enableHeadingRetention() {
        this.enableHeadingRetention = true;
    }

    public void disableHeadingRetention() {
        this.enableHeadingRetention = false;
    }

    @Override
    public void onOpmodeStarted() {
        this.robotInstance = RobotEx.getInstance();
        this.localizer = robotInstance.localizer;
    }

    @Override
    public void onCyclePassed() {

        this.headingPID.updateCoefficients(Drivetrain.kP, Drivetrain.kI, Drivetrain.kD, 0);

/*        RobotOrientation currentOrientation = this.internalIMU.getCurrentFrameRobotOrientation();
        RobotOrientation startOrientation = this.internalIMU.getStartFrameRobotOrientation();

        double tiltError = currentOrientation.getTilt() - startOrientation.getTilt();
        double yawError = currentOrientation.getYaw() - startOrientation.getYaw();


        if (this.enableAntiTip && tiltError > InternalIMU.TILT_THRESHOLD || this.enableAntiTip && yawError > InternalIMU.YAW_THRESHOLD) {
            double tiltOutput = this.tiltPID.getOutputFromError(startOrientation.getTilt(), currentOrientation.getTilt());
            double yawOutput = this.yawPID.getOutputFromError(startOrientation.getYaw(), currentOrientation.getYaw());

            telemetry.addData("Tilt Error: ", tiltError);
            telemetry.addData("Yaw Error: ", yawError);

            telemetry.addData("Tilt Output: ", tiltOutput);
            telemetry.addData("Yaw Output: ", yawOutput);

*//*            leftBackPower -= yawOutput;
            leftFrontPower += yawOutput;
            rightBackPower += yawOutput;
            rightFrontPower -= yawOutput;

            leftBackPower += tiltOutput;
            leftFrontPower += tiltOutput;
            rightBackPower += tiltOutput;
            rightFrontPower += tiltOutput;*//*
        }*/

        rightBackPower *= this.weight;
        rightFrontPower *= this.weight;
        leftBackPower *= this.weight;
        leftFrontPower *= this.weight;

        /*
        if (lastLeftBackPower == 0 && lastLeftFrontPower == 0 && lastRightBackPower == 0 && lastRightFrontPower == 0 && leftBackPower == 0 && leftFrontPower == 0 && rightBackPower == 0 && rightFrontPower == 0) {

        } else {
            this.rightBackMotor.setPower(rightBackPower);
            this.rightFrontMotor.setPower(rightFrontPower);
            this.leftBackMotor.setPower(leftBackPower);
            this.leftFrontMotor.setPower(leftFrontPower);
        }

         */

        this.rightBackMotor.setPower(rightBackPower);
        this.rightFrontMotor.setPower(rightFrontPower);
        this.leftBackMotor.setPower(leftBackPower);
        this.leftFrontMotor.setPower(leftFrontPower);

        this.lastRightBackPower = this.rightBackPower;
        this.lastLeftBackPower= this.leftBackPower;
        this.lastLeftFrontPower = this.leftFrontPower;
        this.lastRightFrontPower = this.rightFrontPower;

        this.rightBackPower = 0;
        this.leftBackPower = 0;
        this.leftFrontPower = 0;
        this.rightFrontPower = 0;

/*        this.telemetry.addData("LF Pos: ", this.leftFrontMotor.getCurrentPosition());
        this.telemetry.addData("LB Pos: ", this.leftBackMotor.getCurrentPosition());
        this.telemetry.addData("RB Pos: ", this.rightBackMotor.getCurrentPosition());
        this.telemetry.addData("RF Pos: ", this.rightFrontMotor.getCurrentPosition());*/

/*        this.telemetry.addData("LF Vel: ", this.leftFrontMotor.getVelocity());
        this.telemetry.addData("LB Vel: ", this.leftBackMotor.getVelocity());
        this.telemetry.addData("RB Vel: ", this.rightBackMotor.getVelocity());
        this.telemetry.addData("RF Vel: ", this.rightFrontMotor.getVelocity());*/
    }

    public void robotCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        // todo: write code for robot centric drive

        /*
        if (this.enableHeadingRetention) {
            if (rightJoystickX == 0) {

                if (lastRot != 0) {
                    this.targetTurnAngle = this.internalIMU.getCurrentFrameHeadingCCW();
                }

                double headingError = MathHelper.getErrorBetweenAngles(
                        this.internalIMU.getCurrentFrameHeadingCCW(),
                        this.targetTurnAngle
                );

                rightJoystickX = headingPID.getOutputFromError(headingError);
            }
        }
*/
        leftJoystickY = -leftJoystickY;

        double multiple = RobotEx.getInstance().getPowerMultiple();

        leftJoystickY = MathHelper.clamp(leftJoystickY * multiple, -1, 1);
        leftJoystickX = MathHelper.clamp(leftJoystickX * multiple, -1, 1);
        rightJoystickX = MathHelper.clamp(rightJoystickX * multiple, -1, 1);

        double denominator = Math.max(Math.abs(leftJoystickY) + Math.abs(leftJoystickX) + Math.abs(rightJoystickX), 1);
        this.leftFrontPower += (leftJoystickY + leftJoystickX + rightJoystickX) / denominator;
        this.leftBackPower += (leftJoystickY - leftJoystickX + rightJoystickX) / denominator;
        this.rightFrontPower += (leftJoystickY - leftJoystickX - rightJoystickX) / denominator;
        this.rightBackPower += (leftJoystickY + leftJoystickX - rightJoystickX) / denominator;

        this.lastX = leftJoystickX;
        this.lastY = leftJoystickY;
        this.lastRot = rightJoystickX;

    }

    public void fieldCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        double currentRobotOrientation = this.localizer.getPose().getHeading() + Math.PI / 2;

        leftJoystickX = -leftJoystickX;

        this.robotCentricDriveFromGamepad(
                Math.sin(currentRobotOrientation) * leftJoystickX + Math.cos(currentRobotOrientation) * leftJoystickY,
                (Math.cos(currentRobotOrientation) * leftJoystickX - Math.sin(currentRobotOrientation) * leftJoystickY),
                rightJoystickX
        );
        // todo: write code for field centric drive
    }

    public void fieldCentricRotationPIDFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickY, double rightJoystickX) {

        double targetAngle = Math.atan2(-rightJoystickX, -rightJoystickY);
        double currentAngle = this.internalIMU.getCurrentFrameHeadingCW();
        double error = targetAngle - currentAngle;

        if (Math.abs(error) > Math.PI) {
            if (targetAngle < 0) {
                targetAngle = AngleHelper.norm(targetAngle);
                error = targetAngle - currentAngle;
            } else if (targetAngle > 0) {
                currentAngle = AngleHelper.norm(currentAngle);
                error = targetAngle - currentAngle;
            }
        }


        if (rightJoystickY == 0 && rightJoystickX == 0) {
            targetAngle = currentAngle;
            error = 0;
        }

        double output = this.headingPID.getOutputFromError(
                error
        );

        this.fieldCentricDriveFromGamepad(
                leftJoystickY,
                leftJoystickX,
                -Math.min(Math.max(output, -0.3), 0.3) + Math.signum(output) * EncoderDrive.kStatic
        );

    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior newZeroPowerBehavior) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setZeroPowerBehavior(newZeroPowerBehavior);
        }
    }

    public void setRunMode(DcMotor.RunMode newRunMode) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setMode(newRunMode);
        }
    }

    public DcMotorEx[] getDrivetrainMotors() {
        return this.drivetrainMotors;
    }

    public MotorGroup<DcMotorEx> getDrivetrainMotorGroup() {
        return this.drivetrainMotorGroup;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return this.leftBackMotor.getZeroPowerBehavior();
    }

    public TurnDirection getTurnDirection() {
        double leftPower = this.leftBackPower + this.leftFrontPower;
        double rightPower = this.rightBackPower + this.rightFrontPower;

        return Math.abs(leftPower) > Math.abs(rightPower) ? TurnDirection.LEFT : TurnDirection.RIGHT;
    }

    public int[] getCWMotorTicks() {
        return new int[] {
                this.rightFrontMotor.getCurrentPosition(),
                this.leftFrontMotor.getCurrentPosition(),
                this.leftBackMotor.getCurrentPosition(),
                this.rightBackMotor.getCurrentPosition()
        };
    }

    public MecanumWheelState getMotorTicks() {
        return new MecanumWheelState(
                this.rightFrontMotor.getCurrentPosition(),
                this.leftFrontMotor.getCurrentPosition(),
                this.leftBackMotor.getCurrentPosition(),
                this.rightBackMotor.getCurrentPosition()
        );
    }

    public MecanumWheelState getMotorVelocity() {
        return new MecanumWheelState(
                this.rightFrontMotor.getVelocity(),
                this.leftFrontMotor.getVelocity(),
                this.leftBackMotor.getVelocity(),
                this.rightBackMotor.getVelocity()
        );
    }

    public ArrayList<Integer> getMotorTicksCCWFromFL() {
        ArrayList<Integer> returnArray = new ArrayList<>();
        returnArray.add(this.leftFrontMotor.getCurrentPosition());
        returnArray.add(this.leftBackMotor.getCurrentPosition());
        returnArray.add(this.rightBackMotor.getCurrentPosition());
        returnArray.add(this.rightFrontMotor.getCurrentPosition());

        return returnArray;
    }

    public ArrayList<Double> getMotorVelocitiesCCWFromFL() {
        ArrayList<Double> returnArray = new ArrayList<>();
        returnArray.add(this.leftFrontMotor.getVelocity());
        returnArray.add(this.leftBackMotor.getVelocity());
        returnArray.add(this.rightBackMotor.getVelocity());
        returnArray.add(this.rightFrontMotor.getVelocity());

        return returnArray;
    }

    public void setWeightedDrivePower(double weight) {
        this.weight = weight;
    }

    public static double getAverageFromArray(int[] array) {
        int sum = 0;

        for (int currentMotorTick : array) {
            sum += currentMotorTick;
        }

        return (double) sum / array.length;

    }

    public double getTrackWidth() {
        return this.trackWidth;
    }

    public double getWheelBase() {
        return this.wheelBase;
    }

    public double getLateralMultiplier() {
        return this.lateralMultiplier;
    }
}


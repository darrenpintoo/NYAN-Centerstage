package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
//boxOpen = 0.3 closed = 0.1
//left servo closed position = 0.55ish  open = 0.80 (better than the 0.85 that we had
//right servo
@Config
public class DepositLift implements Subsystem{

    public enum LiftStates {
        LEVEL0,
        LEVEL1,
        LEVEL2,
        LEVEL3
    }

    public enum BoxStates {
        OPEN,
        CLOSED
    }

    public enum TiltStates {
        DEFAULT,
        TILTED
    }
    double power;

    public DcMotorEx backLiftMotor;
    public DcMotorEx frontLiftMotor;
    public Servo leftServo;
    public Servo rightServo;
    public Servo outtakeServo;

    public Servo boxServo;

    private MotorGroup<DcMotorEx> liftMotors;

    private LiftStates currentTargetState = LiftStates.LEVEL0;
    private LiftStates previousTargetState = LiftStates.LEVEL0;

    private BoxStates boxState = BoxStates.CLOSED;
    private TiltStates tiltState = TiltStates.DEFAULT;
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.15;
    // public static int targetPosition;
    private GeneralPIDController controller = new GeneralPIDController(0, 0, 0, 0);
    public static double leftServoDefaultPosition = 0.43;
    public static double leftServoTiltPosition = 0.73;

    public static double rightServoDefaultPosition = 0.5;
    public static double rightServoTiltPosition = 0.3;
    //
    public static double boxOpenPosition = 0.3;
    public static double boxClosedPosition = 0.6;

    private boolean override = false;
    public int offset = 1;
    public static int offsetLength = 25;

    private Telemetry t;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.backLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backDepositMotor");
        this.frontLiftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontDepositMotor");
        this.leftServo = hardwareMap.get(Servo.class, "LeftBox");
        this.rightServo = hardwareMap.get(Servo.class, "RightBox");
        this.outtakeServo = hardwareMap.get(Servo.class, "BoxOpenServo");

        this.liftMotors = new MotorGroup<>(backLiftMotor, frontLiftMotor);

        this.boxServo = hardwareMap.get(Servo.class, "BoxOpenServo");

        t = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

        this.backLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.backLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.frontLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.backLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void driveLiftFromGamepad(double power) {
        this.power = power;
    }
    @Override
    public void onCyclePassed() {

        controller.updateCoefficients(kP, kI, kD, kF);


        int targetPosition = this.getTargetPositionFromState(this.currentTargetState) + offset * offsetLength;

        if (power == 0) {
            power = MathHelper.clamp(controller.getOutputFromError(targetPosition, this.frontLiftMotor.getCurrentPosition()), -0.5, 0.8);
        }

        t.addData("Power:", power);
        t.addData("Current Position: ", this.frontLiftMotor.getCurrentPosition());
        t.addData("Target Position: ", targetPosition);
        t.addData("leftServo Position: ", leftServo.getPosition());
        t.addData("Tilted: ", this.tiltState == TiltStates.TILTED);

        this.liftMotors.setPower(power);

        if (this.previousTargetState != this.currentTargetState) {
            this.setOffset(0);
        }

        if (this.previousTargetState != this.currentTargetState && this.currentTargetState == LiftStates.LEVEL0) {
            this.setBoxState(BoxStates.CLOSED);
            this.setTiltState(TiltStates.DEFAULT);
        } else if (this.currentTargetState != LiftStates.LEVEL0) {
            if (!override) {
                this.setTiltState(TiltStates.TILTED);
            }
        }

        if (this.tiltState == TiltStates.TILTED) {
            this.leftServo.setPosition(leftServoTiltPosition);
            this.rightServo.setPosition(rightServoTiltPosition);
            t.addData("Tilted", 1);
        } else {
            this.leftServo.setPosition(leftServoDefaultPosition);
            this.rightServo.setPosition(rightServoDefaultPosition);
            t.addData("Not Tilted", 1);
        }
        this.boxServo.setPosition(this.getBoxPositionFromState(this.boxState));


        this.override = false;
        power = 0;
    }

    public int getTargetPositionFromState(LiftStates state) {
        switch (state) {
            case LEVEL0:
                return -10;
            case LEVEL1:
                return 430;
            case LEVEL2:
                return 700;
            case LEVEL3:
                return 1000;
            default:
                return 1000;
        }
    }

    public double getBoxPositionFromState(BoxStates state) {
        switch (state) {
            case OPEN:
                return boxOpenPosition;
            case CLOSED:
                return boxClosedPosition;
            default:
                return 0;
        }
    }

    public void setBoxState(BoxStates state) {
        this.boxState = state;
    }

    public void setTiltState(TiltStates state) {
        this.tiltState = state;
        this.override = true;
    }
    public void setTargetState(LiftStates state) {

        if (state == LiftStates.LEVEL0) {
            this.setTiltState(TiltStates.DEFAULT);
        }
        this.previousTargetState = this.currentTargetState;
        this.currentTargetState = state;
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }
    public void incrementOffset(int sign) {
        this.offset += sign;
    }
}
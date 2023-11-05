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
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.15;
    // public static int targetPosition;
    private GeneralPIDController controller = new GeneralPIDController(0, 0, 0, 0);
    public static double leftServoDefaultPosition = 0.80;
    public static double leftServoTiltPosition = 0.55;
    //
    public static double boxOpenPosition = 0.3;
    public static double boxClosedPosition = 0.1;

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


        int targetPosition = this.getTargetPositionFromState(this.currentTargetState);

        if (power == 0) {
            power = MathHelper.clamp(controller.getOutputFromError(targetPosition, this.frontLiftMotor.getCurrentPosition()), -0.5, 0.5);
        }

        t.addData("Power:", power);
        t.addData("Current Position: ", this.frontLiftMotor.getCurrentPosition());
        t.addData("Target Position: ", targetPosition);
        this.liftMotors.setPower(power);

        if (this.previousTargetState != this.currentTargetState && this.currentTargetState == LiftStates.LEVEL0) {
            this.setBoxState(BoxStates.CLOSED);
        }

        this.boxServo.setPosition(this.getBoxPositionFromState(this.boxState));

        power = 0;
    }

    public int getTargetPositionFromState(LiftStates state) {
        switch (state) {
            case LEVEL0:
                return 0;
            case LEVEL1:
                return 400;
            case LEVEL2:
                return 700;
            case LEVEL3:
                return 1200;
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
    public void setTargetState(LiftStates state) {
        this.previousTargetState = this.currentTargetState;
        this.currentTargetState = state;
    }
}
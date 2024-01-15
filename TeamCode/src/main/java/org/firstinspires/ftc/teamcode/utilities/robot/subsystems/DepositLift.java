package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.MotionProfiledMotion;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
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

    public BoxStates boxState = BoxStates.CLOSED;
    private TiltStates tiltState = TiltStates.DEFAULT;
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0.001;
    public static double kF = 0.1;
    public static double kV = 0.0003;
    public static double kA = 0;
    public static double vMax = 4000;
    public static double aMax = 3000;
    // public static int targetPosition;
    private GeneralPIDController controller = new GeneralPIDController(0, 0, 0, 0);
    public static double leftServoDefaultPosition = 0.45;
    public static double rightServoDefaultPosition = 0.55;
    public static double tiltAmount = 0.28;
    //
    public static double boxOpenPosition = 0.3;
    public static double boxClosedPosition = 0.6;

    public ElapsedTime timer = new ElapsedTime();
    private boolean override = false;
    public int offset = 0;
    public static int offsetLength = 100;

    public static int position = 900;

    private ElapsedTime boxCloseTimer = new ElapsedTime();

    private MotionProfiledMotion profile = new MotionProfiledMotion(
            new MotionProfile(0, 0, vMax, aMax),
            new GeneralPIDController(kP, kI, kD, kF)
    );
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

        profile.setPIDCoefficients(kP, kI, kD, kF);
        profile.setProfileCoefficients(kV, kA, vMax, aMax);

        if (power == 0) {
            power = profile.getOutput(this.frontLiftMotor.getCurrentPosition());
        }

        if (atTargetPosition() && this.currentTargetState == LiftStates.LEVEL0) {
            power /= 2;
        }
        /*
        t.addData("Power:", power);
        t.addData("{Current}/{Target}: ", this.frontLiftMotor.getCurrentPosition() + "/" + this.getTargetPositionFromState(currentTargetState));
        t.addData("leftServo Position: ", leftServo.getPosition());
        t.addData("Offset: ", this.offset);

         */

        if (this.previousTargetState != this.currentTargetState) {
            this.setOffset(0);
        }

        if (this.previousTargetState != this.currentTargetState && this.currentTargetState == LiftStates.LEVEL0) {
            this.setBoxState(BoxStates.CLOSED);
            this.setTiltState(TiltStates.DEFAULT);
        } else if (this.currentTargetState != LiftStates.LEVEL0) {
            if (!override && atTargetPosition()) {
                this.setTiltState(TiltStates.TILTED);
            }
        }

        if (this.tiltState == TiltStates.TILTED) {
            this.leftServo.setPosition(leftServoDefaultPosition+tiltAmount);
            this.rightServo.setPosition(rightServoDefaultPosition-tiltAmount);
        } else {
            if (this.override && this.boxCloseTimer.seconds() < 0.4 && this.currentTargetState != LiftStates.LEVEL0) {
                this.leftServo.setPosition(leftServoDefaultPosition+tiltAmount);
                this.rightServo.setPosition(rightServoDefaultPosition-tiltAmount);
            } else {
                this.leftServo.setPosition(leftServoDefaultPosition);
                this.rightServo.setPosition(rightServoDefaultPosition);
            }
        }
        this.previousTargetState = currentTargetState;
        this.override = false;
        this.liftMotors.setPower(power);
        this.boxServo.setPosition(getBoxPositionFromState(this.boxState));
        power = 0;
    }

    public int getTargetPositionFromState(LiftStates state) {
        int pos;

        switch (state) {
            case LEVEL0:
                pos = -10;
                break;
            case LEVEL1:
                pos = 430;
                break;
            case LEVEL2:
                pos = 700;
                break;
            case LEVEL3:
                pos = position;
            default:
                pos = 1000;
        }

        return pos + offset * offsetLength;
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
        if (this.boxState == state) { return; }
        this.boxState = state;
        boxCloseTimer.reset();
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

        this.regenerateProfile();

    }

    public void setOffset(int offset) {
        if (this.offset == offset) {return;}
        this.offset = offset;

        this.regenerateProfile();
    }
    public void incrementOffset(int sign) {
        this.offset += sign;

        this.regenerateProfile();
    }

    public boolean atTargetPosition() {
        return this.profile.atTargetPosition();
    }

    private void regenerateProfile() {
        timer.reset();

        this.profile.updateTargetPosition(getTargetPositionFromState(this.currentTargetState), this.frontLiftMotor.getCurrentPosition());
    }
}
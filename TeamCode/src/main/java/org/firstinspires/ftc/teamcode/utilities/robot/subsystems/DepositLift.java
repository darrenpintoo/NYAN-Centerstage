package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.MotionProfiledMotion;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

//boxOpen = 0.3 closed = 0.1
//left servo closed position = 0.55ish  open = 0.80 (better than the 0.85 that we had
//right servo
@Config
public class DepositLift implements Subsystem{

    public enum LiftStates {
        LEVEL0,
        LEVEL1,
        LEVEL1_AUTO,
        LEVEL2,
        LEVEL3,
        CUSTOM
    }

    public enum BoxStates {
        OPEN,
        CLOSED
    }

    public enum TiltStates {
        DEFAULT,
        TILTED
    }

    public enum PaintbrushStates {
        ACTIVATED(0),
        DEFAULT(1);

        final double position;
        PaintbrushStates(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    double power;

    public DcMotorEx backLiftMotor;
    public DcMotorEx frontLiftMotor;
    public Servo leftServo;
    public Servo rightServo;
    public Servo outtakeServo;

    public Servo boxServo;

    public Servo paintbrushServo;

    private MotorGroup<DcMotorEx> liftMotors;

    private LiftStates currentTargetState = LiftStates.LEVEL0;
    private LiftStates previousTargetState = LiftStates.LEVEL0;
    public BoxStates boxState = BoxStates.CLOSED;
    private TiltStates tiltState = TiltStates.DEFAULT;

    private PaintbrushStates paintbrushState = PaintbrushStates.DEFAULT;
    private DigitalChannel magneticLimitSwitch;

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
    public static double leftServoDefaultPosition = 0.54;
    public static double rightServoDefaultPosition = 0.46;
    public static double tiltAmount = -0.28;
    public static double normalAmount = -0.07;
    //
    public static double boxOpenPosition = 0.6;
    public static double boxClosedPosition = 1;

    public ElapsedTime timer = new ElapsedTime();
    private boolean override = false;
    private boolean flutter = false;
    private ElapsedTime flutterTimer = new ElapsedTime();
    public double offset = 0;
    public static int offsetLength = 100;

    public static int position = 900;
    public int customPosition = 0;
    public static double flutterTime = 0.2;
    public static double paintbrushPosition = 0;

    private ElapsedTime boxCloseTimer = new ElapsedTime();
    private ElapsedTime frameTime = new ElapsedTime();

    private MotionProfiledMotion profile = new MotionProfiledMotion(
            new MotionProfile(0, 0, vMax, aMax),

            new GeneralPIDController(kP, kI, kD, kF)
    );
    private Telemetry t;

    private int startPosition = 0;
    private int currentSlidesPosition = 0;

    private ElapsedTime onTimer;
    private boolean lastSwitchState = true;
    private boolean currentSwitchState = false;
    private boolean onLastTime = false;
    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.backLiftMotor = new CachingDcMotorEX((DcMotorEx) hardwareMap.get(DcMotor.class, "backDepositMotor"), 0.0025);
        this.frontLiftMotor = new CachingDcMotorEX((DcMotorEx) hardwareMap.get(DcMotor.class, "frontDepositMotor"), 0.0025);
        this.leftServo = new CachingServo(hardwareMap.get(Servo.class, "leftBox"), 1e-3);
        this.rightServo = new CachingServo(hardwareMap.get(Servo.class, "rightBox"), 1e-3);
        this.outtakeServo = new CachingServo(hardwareMap.get(Servo.class, "boxOpenServo"), 1e-3);

        this.paintbrushServo = new CachingServo(hardwareMap.get(Servo.class, "paintbrush"), 1e-3);
        this.liftMotors = new MotorGroup<>(backLiftMotor, frontLiftMotor);

        this.boxServo = new CachingServo(hardwareMap.get(Servo.class, "boxOpenServo"), 1e-3);
        this.magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");

        this.onTimer = new ElapsedTime();

        this.previousTargetState = LiftStates.LEVEL0;
        this.currentTargetState = LiftStates.LEVEL0;
        this.setTargetState(LiftStates.LEVEL0);

        this.previousTargetState = LiftStates.LEVEL0;
        this.currentTargetState = LiftStates.LEVEL0;
        this.regenerateProfile();

        t = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

        this.backLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.backLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // this.frontLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.backLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        this.previousTargetState = LiftStates.LEVEL0;
        this.currentTargetState = LiftStates.LEVEL0;
        this.setTargetState(LiftStates.LEVEL0);

        this.previousTargetState = LiftStates.LEVEL0;
        this.currentTargetState = LiftStates.LEVEL0;
        this.regenerateProfile();

        this.setTiltState(TiltStates.DEFAULT);


        this.magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        this.setBoxState(BoxStates.CLOSED);

    }

    public void driveLiftFromGamepad(double power) {
        this.power = power;
    }
    @Override
    public void onCyclePassed() {
        frameTime.reset();


        profile.setPIDCoefficients(kP, kI, kD, kF);
        profile.setProfileCoefficients(kV, kA, vMax, aMax);


        currentSlidesPosition = getPosition();
        currentSwitchState = magneticLimitSwitch.getState();

        // t.addData("Deposit -1: ", frameTime.milliseconds());

        if (atTargetPosition() && this.currentTargetState == LiftStates.LEVEL0 && this.offset == 0) {

            power /= 2;


            if (currentSwitchState && power == 0 && this.profile.timer.seconds() - this.profile.feedforwardProfile.getDuration() < 0.5) {
               power = -0.4;
            }
        }

        if (!currentSwitchState) {
            startPosition = currentSlidesPosition;
        }

        this.lastSwitchState = currentSwitchState;

        if (power == 0) {
            int a = currentSlidesPosition - startPosition;
            // t.addData("Deposit 0: ", frameTime.milliseconds());

            power = profile.getOutput(a);
            // t.addData("Deposit 1: ", frameTime.milliseconds());

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
            if (this.override && this.boxCloseTimer.milliseconds() < 0.4 && this.currentTargetState != LiftStates.LEVEL0) {
                this.leftServo.setPosition(leftServoDefaultPosition+tiltAmount);
                this.rightServo.setPosition(rightServoDefaultPosition-tiltAmount);
            } else {
                this.leftServo.setPosition(leftServoDefaultPosition+normalAmount);
                this.rightServo.setPosition(rightServoDefaultPosition-normalAmount);
            }
        }

        this.previousTargetState = currentTargetState;
        this.override = false;
        this.liftMotors.setPower(power);
        if (flutter && flutterTimer.seconds() > flutterTime) {
            this.setBoxState(BoxStates.CLOSED);
            flutter = false;
        }

        this.boxServo.setPosition(getBoxPositionFromState(this.boxState));
        this.paintbrushServo.setPosition(paintbrushState.getPosition());
        // t.addData("IsDown?: ", currentSwitchState);
        t.addData("Slides Position: ", currentSlidesPosition);
        t.addData("Custom Position: ", customPosition);
        t.addData("State: ", currentTargetState);
        power = 0;
    }

    public double getTargetPositionFromState(LiftStates state) {
        int pos;

        switch (state) {
            case LEVEL0:
                pos = -10;
                break;
            case LEVEL1:
                pos = 430;
                break;
            case LEVEL1_AUTO:
                pos = 550;
                break;
            case LEVEL2:
                pos = 700;
                break;
            case LEVEL3:
                pos = position;
                break;
            case CUSTOM:
                pos = customPosition;
                break;
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
        flutter = false;
        flutterTimer.reset();
    }

    public void setTiltState(TiltStates state) {
        this.tiltState = state;
        this.override = true;
    }

    public void flutterBox() {
        this.setBoxState(BoxStates.OPEN);
        flutter = true;
        flutterTimer.reset();
    }
    public void setTargetState(LiftStates state) {

        if (state == LiftStates.LEVEL0) {
            this.setTiltState(TiltStates.DEFAULT);
        }
        this.previousTargetState = this.currentTargetState;
        this.currentTargetState = state;
        this.offset = 0;
        this.regenerateProfile();

    }

    public void setOffset(int offset) {
        if (this.offset == offset) {return;}
        this.offset = offset;

        this.regenerateProfile();
    }
    public void incrementOffset(double sign) {
        this.offset += sign;

        this.regenerateProfile();
    }

    public boolean atTargetPosition() {
        return this.profile.atTargetPosition();
    }

    private void regenerateProfile() {
        timer.reset();

        this.profile.updateTargetPosition(getTargetPositionFromState(this.currentTargetState), currentSlidesPosition);
    }

    public void reset() {
        this.liftMotors.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotors.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPaintbrushState(PaintbrushStates newState) {
        this.paintbrushState = newState;
    }

    public void setCustomPosition(int newPosition) {
        this.customPosition = newPosition;
    }

    public int getPosition() {
        return this.frontLiftMotor.getCurrentPosition();
    }
}
package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.DigitalCommands;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Intake implements Subsystem {

    Servo leftRotationServo;
    Servo rightRotationServo;
    public Servo intakeClawServo;

    DigitalChannel breakBeam;

    DigitalChannel centerProximity;
    DigitalChannel leftProximity;
    DigitalChannel rightProximity;

    AnalogInput intakeAnaglog;

    Rev2mDistanceSensor left;
    Rev2mDistanceSensor right;


    public enum RotationStates {
        DEFAULT,
        FULL_DEFAULT,
        ROTATED
    }

    public enum GripperStates {
        OPEN,
        CLOSED
    }
    // public static double startRotationPosition = 0.23;
    // public static double endRotationPosition = 0.84;

    public static double openClawPosition = 0.44;
    public static double partlyOpenClawPosition = 0.58;
    public static double closeClawPosition = 0.63;
    // public static double num = 0;

    private double offset = 0;
    public static double offsetLength = 0.028;

    public RotationStates currentRotationState = RotationStates.DEFAULT;
    public GripperStates currentGripperState = GripperStates.OPEN;
    Telemetry t;

    // public static double defaultPosition = 0.2;
    // public static double placingPosition = 0.76;

    public static double activatedRotationOffset = 0.78;
    public static double fullIntakeRotationOffset = 0.14;
    public static double intakeRotationOffset = 0.205;

    public static double aMax = 3;
    public static double vMax = 3;

    private MotionProfile profile;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime clawTimer = new ElapsedTime();
    boolean waitingForAction = false;

    boolean inTeleop = false;


    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        leftRotationServo = new CachingServo(hardwareMap.get(Servo.class, "leftRotationServo"), 1e-3);
        rightRotationServo = new CachingServo(hardwareMap.get(Servo.class, "rightRotationServo"), 1e-3);
        intakeClawServo = new CachingServo(hardwareMap.get(Servo.class, "intakeClaw"), 1e-3);

        breakBeam = hardwareMap.get(DigitalChannel.class, "intakeBreakBeam");
        centerProximity = hardwareMap.get(DigitalChannel.class, "intakeCenterProximity");
        leftProximity = hardwareMap.get(DigitalChannel.class, "intakeLeftProximity");
        rightProximity = hardwareMap.get(DigitalChannel.class, "intakeRightProximity");

        intakeAnaglog = hardwareMap.get(AnalogInput.class, "intakeAnalog");

        left = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "leftDistance");
        right = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "rightDistance");


        this.t = telemetry;

        breakBeam.setMode(DigitalChannel.Mode.INPUT);
        centerProximity.setMode(DigitalChannel.Mode.INPUT);
        leftProximity.setMode(DigitalChannel.Mode.INPUT);
        rightProximity.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void onOpmodeStarted() {
        profile = new MotionProfile(
                getRotationPosition(this.currentRotationState),
                getRotationPosition(this.currentRotationState),
                vMax,
                aMax
                );

        timer.reset();
    }

    @Override
    public void onCyclePassed() {
        double position = getCurrentPosition();
        this.intakeClawServo.setPosition(getClawPosition());
        this.leftRotationServo.setPosition(position);
        this.rightRotationServo.setPosition(position);


        /*
        t.addData("Intake Servo Position: ", rightRotationServo.getPosition());
        t.addData("Intake Servo Position: ", leftRotationServo.getPosition());

        t.addData("Break beam: ", breakBeam.getState());
        t.addData("Proximity c: ", centerProximity.getState());
        t.addData("Proximity r: ", rightProximity.getState());
        t.addData("Proximity l: ", leftProximity.getState());

         */
        // t.addData("Timer: ", timer.seconds());
        // t.addData("Open?: ", this.currentGripperState);
        // t.addData("Rot: ", this.currentRotationState);
        // t.addData("offset: ", offset);


        if (!breakBeam.getState() && currentGripperState == GripperStates.OPEN && clawTimer.seconds() > 0.5 && offset == 0) {
            this.setGripperState(GripperStates.CLOSED);
        }

        /*
        if (inTeleop && !centerProximity.getState() && offset > 0 && currentGripperState == GripperStates.OPEN) {
            this.setGripperState(GripperStates.CLOSED);
        }
         */

        // TODO: Sync with teleop
        if (clawTimer.seconds() > 0.25 && currentGripperState == GripperStates.CLOSED && currentRotationState == RotationStates.FULL_DEFAULT && !waitingForAction) {
            this.incrementOffset(2);
            waitingForAction = true;
        }

        if (currentGripperState != GripperStates.CLOSED && waitingForAction) {
            this.setOffset(0);

            if ((timer.seconds()-0.5 > this.profile.getDuration())) {
                this.setGripperState(GripperStates.CLOSED);
                waitingForAction = false;
            }
        }

        if (clawTimer.seconds() > 0.3 && currentRotationState == RotationStates.ROTATED && currentGripperState != GripperStates.CLOSED) {
            this.setRotationState(RotationStates.DEFAULT);
        }
        if (this.currentRotationState == RotationStates.DEFAULT && (timer.seconds() > this.profile.getDuration())) {

            if (timer.seconds()+1.5 > this.profile.getDuration()) {
                this.setRotationState(RotationStates.FULL_DEFAULT);
                this.setGripperState(GripperStates.OPEN);
            }
        }

        t.addData("Analog: ", intakeAnaglog.getVoltage());
        t.addData("Analog: ", intakeAnaglog.getVoltage());

    }

    public void reset() {
        this.setGripperState(GripperStates.OPEN);
        this.setRotationState(RotationStates.FULL_DEFAULT);

        this.setOffset(0);

        waitingForAction = false;
    }
    public void setGripperState(GripperStates newGripState) {
        clawTimer.reset();
        this.currentGripperState = newGripState;
    }

    public double getRotationPosition(RotationStates currentRotationState)  {

        double pos;

        switch (currentRotationState) {
            case ROTATED:
                pos = activatedRotationOffset;
                break;
            case DEFAULT:
                pos = intakeRotationOffset;
                break;
            case FULL_DEFAULT:
                pos = fullIntakeRotationOffset;
                break;
            default:
                pos = 0;
        }

        return pos + offset * offsetLength;
    }

    public double getClawPosition() {
        switch (currentGripperState) {
            case OPEN:
            {
                if (this.currentRotationState == RotationStates.ROTATED) {
                    return partlyOpenClawPosition;
                } else {
                        return openClawPosition;
                    }
                }
            case CLOSED:
                return closeClawPosition;
        }

        return 0;
    }
    public void setRotationState(RotationStates newRotationState) {

        if (this.currentRotationState != newRotationState) {
            this.setOffset(0);
            this.setGripperState(GripperStates.CLOSED);
            waitingForAction = false;
        }

        this.currentRotationState = newRotationState;

        this.rebuildProfile();
    }

    public double getCurrentPosition() {
        return this.profile.getPositionFromTime(timer.seconds());
    }
    public void incrementOffset(int sign) {
        offset += sign;

        MathHelper.clamp(offset, 0, Integer.MAX_VALUE);

        this.rebuildProfile();
    }

    private void rebuildProfile() {
        this.profile = new MotionProfile(
                getCurrentPosition(),
                getRotationPosition(currentRotationState),
                vMax,
                aMax
        );

        timer.reset();

    }

    public void setOffset(double offset) {
        if (this.offset == offset) { return;}
        this.offset = offset;

        this.rebuildProfile();
    }

    public boolean getLeftProximity() {
        return !leftProximity.getState();
    }

    public boolean getRightProximity() {
        return !rightProximity.getState();
    }

    public boolean getCenterProximity() {
        return !centerProximity.getState();
    }

    public void enableTeleop() {
        this.inTeleop = true;
    }

    public void disableTeleop()  {
        this.inTeleop = false;
    }

    public boolean rightClear() {
        return right.getDistance(DistanceUnit.INCH) > 25;
    }

    public boolean leftClear() {
        return left.getDistance(DistanceUnit.INCH) > 25;
    }


}

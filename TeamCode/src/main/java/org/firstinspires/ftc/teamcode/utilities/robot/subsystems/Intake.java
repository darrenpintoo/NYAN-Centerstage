package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;

@Config
public class Intake implements Subsystem {

    Servo leftRotationServo;
    Servo rightRotationServo;
    Servo intakeClawServo;

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

    public static double openClawPosition = 0.35;
    public static double partlyOpenClawPosition = 0.45;
    public static double closeClawPosition = 0.5;
    // public static double num = 0;

    private int offset = 0;
    public static double offsetLength = 0.015;

    public RotationStates currentRotationState = RotationStates.DEFAULT;
    public GripperStates currentGripperState = GripperStates.OPEN;
    Telemetry t;

    // public static double defaultPosition = 0.2;
    // public static double placingPosition = 0.76;

    public static double activatedRotationOffset = 0.69;
    public static double fullIntakeRotationOffset = 0.1;
    public static double intakeRotationOffset = 0.2;

    public static double aMax = 1;
    public static double vMax = 1;

    private MotionProfile profile;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime clawTimer = new ElapsedTime();
    boolean waitingForAction = false;



    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        // leftRotationServo = hardwareMap.get(Servo.class, "leftRotationServo");
        rightRotationServo = hardwareMap.get(Servo.class, "rightRotationServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClaw");

        this.t = telemetry;
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
        // this.leftRotationServo.setPosition(position);
        this.rightRotationServo.setPosition(position);

        /*
        t.addData("Intake Servo Position: ", position);
        t.addData("Timer: ", timer.seconds());
        t.addData("Open?: ", this.currentGripperState);
        t.addData("Rot: ", this.currentRotationState);
        t.addData("offset: ", offset);*/

        // TODO: Sync with teleop
        if (clawTimer.seconds() > 0.5 && currentGripperState == GripperStates.CLOSED && currentRotationState == RotationStates.FULL_DEFAULT && !waitingForAction) {
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

        if (clawTimer.seconds() > 0.5 && currentRotationState == RotationStates.ROTATED && currentGripperState != GripperStates.CLOSED) {
            this.setRotationState(RotationStates.DEFAULT);
        }
        if (this.currentRotationState == RotationStates.DEFAULT && (timer.seconds() > this.profile.getDuration())) {

            if (timer.seconds()+1.5 > this.profile.getDuration()) {
                this.setRotationState(RotationStates.FULL_DEFAULT);
                this.setGripperState(GripperStates.OPEN);
            }
        }
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

    public void setOffset(int offset) {
        if (this.offset == offset) { return;}
        this.offset = offset;

        this.rebuildProfile();
    }

}

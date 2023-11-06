package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;

@Config
public class Intake implements Subsystem {

    Servo gripServo;
    Servo rotationServo;

    public enum RotationStates {
        DEFAULT,
        ROTATED
    }

    public enum GripperStates {
        OPEN,
        CLOSED
    }
    public static double startRotationPosition = 0.23;
    public static double endRotationPosition = 0.84;

    public static double openClawPosition = 0;
    public static double closeClawPosition = 0.25;
    public static double num = 0;

    int offset = 0;
    public static double offsetLength = 0.015;

    private RotationStates currentRotationState = RotationStates.DEFAULT;
    private GripperStates currentGripperState = GripperStates.OPEN;
    Telemetry t;


    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        gripServo = hardwareMap.get(Servo.class, "IntakeGripServo");
        rotationServo = hardwareMap.get(Servo.class, "IntakeRotationServo");

        this.t = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

        double currentClawPosition;
        double currentRotationPosition;

        if (currentGripperState == GripperStates.OPEN) {
            currentClawPosition = openClawPosition;
        } else {
            currentClawPosition = closeClawPosition;
        }

        if (currentRotationState == RotationStates.DEFAULT) {
            currentRotationPosition = startRotationPosition;
        } else {
            currentRotationPosition = endRotationPosition;
        }

        currentRotationPosition = currentRotationPosition + offset*offsetLength;
        this.rotationServo.setPosition(currentRotationPosition);
        this.gripServo.setPosition(currentClawPosition);

        t.addData("Position Rot: ", currentRotationPosition);
        t.addData("Grip Position: ", currentClawPosition);
    }

    public void setGripperState(GripperStates newGripState) {
        this.currentGripperState = newGripState;
    }

    public void setRotationState(RotationStates newRotationState) {

        if (this.currentRotationState != newRotationState) {
            this.setOffset(0);
        }
        this.currentRotationState = newRotationState;
    }

    public void incrementOffset(int sign) {
        this.offset += sign;

        MathHelper.clamp(this.offset, 0, Integer.MAX_VALUE);
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }

}

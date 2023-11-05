package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private RotationStates currentRotationState = RotationStates.DEFAULT;
    private GripperStates currentGripperState = GripperStates.OPEN;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        gripServo = hardwareMap.get(Servo.class, "IntakeGripServo");
        rotationServo = hardwareMap.get(Servo.class, "IntakeRotationServo");
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {
        if (currentGripperState == GripperStates.OPEN) {
            gripServo.setPosition(openClawPosition);
        } else if (currentGripperState == GripperStates.CLOSED) {
            gripServo.setPosition(closeClawPosition);
        }

        if (currentRotationState == RotationStates.DEFAULT) {
            rotationServo.setPosition(startRotationPosition);
        } else if (currentRotationState == RotationStates.ROTATED) {
            rotationServo.setPosition(endRotationPosition);
        }
    }

    public void setGripperState(GripperStates newGripState) {
        this.currentGripperState = newGripState;
    }

    public void setRotationState(RotationStates newRotationState) {
        this.currentRotationState = newRotationState;
    }

}

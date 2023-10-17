package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake implements Subsystem {

    Servo gripServo;
    Servo rotationServo;

    public static double num = 0.5;
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
        rotationServo.setPosition(num);
    }
}

package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class PlaneLauncher implements Subsystem {

    public enum AirplaneLiftStates {
        DOWN,
        UP
    }

    public enum AirplaneShootStates {
        CLOSED,
        OPENED
    }
    Servo droneAngleServo;
    Servo droneLaunchServo;

    public static double angleHome = 0.8;
    public static double angleActivated = 0.61;
    public static double launchClosed = 0.8;
    public static double launchOpen = 0.65;

    private AirplaneShootStates currentShootState;
    private AirplaneLiftStates currentLiftState;


    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        droneAngleServo = new CachingServo(hardwareMap.get(Servo.class, "DroneAngle"), 1e-4);
        droneLaunchServo = new CachingServo(hardwareMap.get(Servo.class, "DroneLaunch"), 1e-4);
    }

    @Override
    public void onOpmodeStarted() {
    }

    public void setShootState(AirplaneShootStates state) {
        this.currentShootState = state;
    }

    public void setLiftState(AirplaneLiftStates state) {
        this.currentLiftState = state;
    }
    @Override
    public void onCyclePassed() {
        droneLaunchServo.setPosition(currentShootState == AirplaneShootStates.OPENED ? launchOpen : launchClosed);
        droneAngleServo.setPosition(currentLiftState == AirplaneLiftStates.UP ? angleActivated : angleHome);
    }
}

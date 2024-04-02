package org.firstinspires.ftc.teamcode.utilities.robot.movement;

public class MovementConstants {

    public double velocityMax;
    public double accelerationMax;
    public double maxCorrectionTime;

    public MovementConstants(
            double velocityMax,
            double accelerationMax,
            double maxCorrectionTime
    ) {
        this.velocityMax = velocityMax;
        this.accelerationMax = accelerationMax;
        this.maxCorrectionTime = maxCorrectionTime;
    }
}

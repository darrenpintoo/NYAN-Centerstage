package org.firstinspires.ftc.teamcode.utilities.robot.movement;

public class MovementConstants {

    public double velocityMax;
    public double accelerationMax;
    public double decelerationMax;

    public double maxCorrectionTime;

    public MovementConstants(
            double velocityMax,
            double accelerationMax,
            double decelerationMax,
            double maxCorrectionTime
    ) {
        this.velocityMax = velocityMax;
        this.accelerationMax = accelerationMax;
        this.decelerationMax = decelerationMax;
        this.maxCorrectionTime = maxCorrectionTime;
    }
}

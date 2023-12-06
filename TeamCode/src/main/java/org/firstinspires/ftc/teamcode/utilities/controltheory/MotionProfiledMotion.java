package org.firstinspires.ftc.teamcode.utilities.controltheory;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;

public class MotionProfiledMotion {

    GeneralPIDController feedbackPositionController;
    public MotionProfile feedforwardProfile;

    public ElapsedTime timer;

    private double kV = 0;
    private double kA = 0;

    private double vMax = 1000;
    private double aMax = 1000;
    public MotionProfiledMotion(MotionProfile feedforward, GeneralPIDController feedback) {
        this.feedforwardProfile = feedforward;
        this.feedbackPositionController = feedback;

        this.timer = new ElapsedTime();
    }

    public double getOutput(double currentPosition) {

        double currentUpdateTime = timer.seconds();

        double targetPosition = this.feedforwardProfile.getPositionFromTime(currentUpdateTime);
        double targetVelocity = this.feedforwardProfile.getVelocityFromTime(currentUpdateTime);
        double targetAcceleration = this.feedforwardProfile.getAccelerationFromTime(currentUpdateTime);

        double feedforward = targetAcceleration * kA + targetVelocity * kV;

        double feedback = this.feedbackPositionController.getOutputFromError(
                targetPosition - currentPosition
        );


        return feedback + feedforward;
    }

    public void setPIDCoefficients(double kP, double kI, double kD, double kF) {
        this.feedbackPositionController.updateCoefficients(
                kP, kI, kD, kF
        );
    }

    public void setProfileCoefficients(double kV, double kA, double vMax, double aMax) {
        this.kV = kV;
        this.kA = kA;
        this.vMax = vMax;
        this.aMax = aMax;
    }

    public void updateTargetPosition(double targetPosition, double currentPosition) {
        this.feedforwardProfile = new MotionProfile(
                currentPosition,
                targetPosition,
                vMax,
                aMax
        );

        this.timer.reset();
    }
}

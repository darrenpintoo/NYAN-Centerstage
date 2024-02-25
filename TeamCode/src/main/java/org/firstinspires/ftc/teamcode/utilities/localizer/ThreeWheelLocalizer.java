package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.BaseEncoder;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;

@Config
public class ThreeWheelLocalizer {

    private Telemetry telemetry;

    private BaseEncoder leftParallel;
    private BaseEncoder rightParallel;
    private BaseEncoder backPerpendicular;

    private InternalIMU imu;

    // Constants for wheel positions
    private static final double WHEEL_BASE = 14.5; // Replace with your robot's wheelbase in inches
    private static final double TRACK_WIDTH = 15.0; // Replace with your robot's track width in inches

    // Variables to store previous encoder values for dead wheel calculations
    private double prevLeftParallelTicks;
    private double prevRightParallelTicks;
    private double prevBackPerpendicularTicks;

    // Variables to store robot pose (x, y, heading)

    private Pose pose = new Pose(0, 0, 0);
    private Pose velocity = new Pose(0, 0, 0);

    public static double trackWidth = -6.9;
    public static double fowardOffset = 2.9;

    private double headingError = 0;

    private double actualHeading = 0;


    ElapsedTime IMUUpdateTimer = new ElapsedTime();
    ElapsedTime updateTimer = new ElapsedTime();

    public ThreeWheelLocalizer(BaseEncoder lp, BaseEncoder rp, BaseEncoder bp, InternalIMU imu, Telemetry telemetry) {
        this.leftParallel = lp;
        this.rightParallel = rp;
        this.backPerpendicular = bp;
        this.telemetry = telemetry;

        this.prevLeftParallelTicks = lp.getTicks();
        this.prevRightParallelTicks = rp.getTicks();
        this.prevBackPerpendicularTicks = bp.getTicks();

        this.imu = imu;

        IMUUpdateTimer.reset();


    }

    public void updatePose() {

        double updateTime = updateTimer.seconds();
        updateTimer.reset();

        double currentLeftParallelTicks = leftParallel.getTicks() * 0.9819;
        double currentRightParallelTicks = rightParallel.getTicks() * 0.9819;
        double currentBackPerpendicularTicks = backPerpendicular.getTicks() * 0.98;

        double deltaLeftParallel = currentLeftParallelTicks - prevLeftParallelTicks;
        double deltaRightParallel = currentRightParallelTicks - prevRightParallelTicks;
        double deltaBackPerpendicular = currentBackPerpendicularTicks - prevBackPerpendicularTicks;

        prevLeftParallelTicks = currentLeftParallelTicks;
        prevRightParallelTicks = currentRightParallelTicks;
        prevBackPerpendicularTicks = currentBackPerpendicularTicks;

        // Convert encoder ticks to inches
        double deltaLeftDistance = (deltaLeftParallel / DriveConstants.TICKS_PER_REVOLUTION) * DriveConstants.WHEEL_SIZE * 2 * Math.PI;
        double deltaRightDistance = (deltaRightParallel / DriveConstants.TICKS_PER_REVOLUTION) * DriveConstants.WHEEL_SIZE * 2 * Math.PI;
        double deltaBackDistance = (deltaBackPerpendicular / DriveConstants.TICKS_PER_REVOLUTION) * DriveConstants.WHEEL_SIZE * 2 * Math.PI;

        double phi = (deltaLeftDistance - deltaRightDistance) / trackWidth;
        phi = imu.getCurrentFrameHeadingCW() - pose.getHeading();
        double deltaMiddle = (deltaLeftDistance + deltaRightDistance) / 2.0;
        double deltaPerpendicular = deltaBackDistance - fowardOffset * phi;
        imu.onCyclePassed();
        phi = imu.getCurrentFrameHeadingCW() - pose.getHeading();
        // x = theta; y = dtheta

        // double arcDeltaX = (Math.sin((phi + deltaPhi)) - Math.sin(phi)) / deltaPhi * delt;

        double sineTerm, cosTerm;

        if (MathHelper.epsilonEquals(phi, 0.0)) {
            sineTerm = 1.0 - phi * phi / 6.0;
            cosTerm = phi / 2.0;
        } else {
            sineTerm = Math.sin(phi) / phi;
            cosTerm = (1 - Math.cos(phi)) / phi;
        }

        Pose twist = new Pose(
                sineTerm * deltaMiddle - cosTerm * deltaPerpendicular,
                cosTerm * deltaMiddle + sineTerm * deltaPerpendicular,
                phi
        );


        actualHeading += phi;
        double deltaX = twist.getX() * Math.sin(pose.getHeading()) - twist.getY() * Math.cos(pose.getHeading());
        double deltaY = twist.getX() * Math.cos(pose.getHeading()) + twist.getY() * Math.sin(pose.getHeading());


        // double deltaX = deltaMiddle * Math.sin(pose.getHeading()) - deltaPerpendicular * Math.cos(pose.getHeading());
        // double deltaY = deltaMiddle * Math.cos(pose.getHeading()) + deltaPerpendicular * Math.sin(pose.getHeading());

        double newPositionX = pose.getX() + deltaX;
        double newPositionY = pose.getY() + deltaY;
        double newHeading = pose.getHeading() + phi;

        // Update robot pose
        pose = new Pose(newPositionX, newPositionY, newHeading);
        velocity = new Pose(deltaX / updateTime, deltaY / updateTime, phi / updateTime);



        // pose.setHeading(imu.getCurrentFrameHeadingCW());



        /*
        if (IMUUpdateTimer.seconds() > 0.25 && velocity.getHeading() < 1) {
            IMUUpdateTimer.reset();
            imu.onCyclePassed();

            double externalHeading = imu.getCurrentFrameHeadingCW();
            // headingError += externalHeading - pose.getHeading();
            pose.setHeading(externalHeading);
        }


         */


        telemetry.addData("Forward Offset: ", deltaBackDistance / phi);
        telemetry.addData("Velocity: ", velocity.getHeading());
        telemetry.addData("Heading Error: ", actualHeading - pose.getHeading());
        /*
        this.telemetry.addData("Perp: ", deltaPerpendicular);
        this.telemetry.addData("Back Distance: ", deltaBackDistance);
        this.telemetry.addData("phi: ", phi);
         */
        /*
        this.telemetry.addData("deltaX: ", deltaX);
        this.telemetry.addData("deltaY: ", deltaY);
        this.telemetry.addData("deltaTheta: ", phi);
         */

        /*
        this.telemetry.addData("lp: ", deltaLeftDistance);
        this.telemetry.addData("rp: ", deltaRightDistance);
        this.telemetry.addData("bp: ", deltaBackDistance);
         */

        // Log telemetry data
        this.telemetry.addData("Robot X (inches): ", this.pose.getX());
        this.telemetry.addData("Robot Y (inches): ", this.pose.getY());
        this.telemetry.addData("Robot Heading: ", this.pose.getHeading());


    }


    public Pose getPose() {
        return pose;
    }

    public void setPose(Pose pose, boolean override) {
        this.pose = pose;
        if (override) {
            imu.setHeadingOffset(pose.getHeading());
            imu.enableHeadingOffsetCorrection();
        }
    }
}
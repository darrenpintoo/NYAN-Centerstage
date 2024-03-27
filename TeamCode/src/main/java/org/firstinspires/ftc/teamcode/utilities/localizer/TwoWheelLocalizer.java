package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.BaseEncoder;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.InternalIMU;


// -0.1 X, 7.1 Y when turning right 90 deg
// -6 X, -1 Y when turning left 90 deg
// -6.5 X, -6.5 Y when turning 180 deg
@Config
public class TwoWheelLocalizer {

    private Telemetry telemetry;

    private BaseEncoder leftParallel;
    private BaseEncoder rightParallel;
    private BaseEncoder backPerpendicular;

    private InternalIMU imu;

    // Variables to store previous encoder values for dead wheel calculations
    private double prevLeftParallelTicks;
    private double prevRightParallelTicks;
    private double prevBackPerpendicularTicks;

    // Variables to store robot pose (x, y, heading)

    private Pose pose = new Pose(0, 0, 0);
    private Pose velocity = new Pose(0, 0, 0);

    public static double trackWidth = 7.05;
    public static double fowardOffset = 5.359;

    private double headingError = 0;

    private double actualHeading = 0;


    private Pose perpendicularPosition = new Pose(0.366, -5.359, 0);
    private Pose parallelPosition = new Pose(3.543, -5.964, Math.PI / 2);

    ElapsedTime IMUUpdateTimer = new ElapsedTime();
    ElapsedTime updateTimer = new ElapsedTime();

    public TwoWheelLocalizer(BaseEncoder lp, BaseEncoder bp, InternalIMU imu, Telemetry telemetry) {
        this.leftParallel = lp;
        this.backPerpendicular = bp;
        this.telemetry = telemetry;

        this.prevLeftParallelTicks = lp.getTicks();
        this.prevBackPerpendicularTicks = bp.getTicks();

        this.imu = imu;

        IMUUpdateTimer.reset();
    }

    public void updatePose() {


        imu.onCyclePassed();

        double currentLeftParallelTicks = leftParallel.getTicks() * 0.9899;
        double currentBackPerpendicularTicks = backPerpendicular.getTicks();

        double updateTime = updateTimer.seconds();
        updateTimer.reset();


        double deltaLeftParallel = currentLeftParallelTicks - prevLeftParallelTicks;
        double deltaBackPerpendicular = currentBackPerpendicularTicks - prevBackPerpendicularTicks;

        prevLeftParallelTicks = currentLeftParallelTicks;
        prevBackPerpendicularTicks = currentBackPerpendicularTicks;

        // Convert encoder ticks to inches
        // double deltaLeftDistance = (deltaLeftParallel / DriveConstants.TICKS_PER_REVOLUTION) * DriveConstants.WHEEL_SIZE * 2 * Math.PI;
        // double deltaBackDistance = (deltaBackPerpendicular / DriveConstants.TICKS_PER_REVOLUTION) * DriveConstants.WHEEL_SIZE * 2 * Math.PI;

        double deltaLeftDistance = deltaLeftParallel * DriveConstants.CONVERSION_CONSTANT;
        double deltaBackDistance = deltaBackPerpendicular * DriveConstants.CONVERSION_CONSTANT;

        double phi = imu.getCurrentFrameHeadingCW() - pose.getHeading();
        double deltaMiddle = deltaLeftDistance + parallelPosition.getX() * phi;
        double deltaPerpendicular = deltaBackDistance + perpendicularPosition.getY() * phi;

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

        double sin = Math.sin(pose.getHeading());
        double cos = Math.cos(pose.getHeading());

        actualHeading += phi;
        double deltaX = twist.getX() * sin - twist.getY() * cos;
        double deltaY = twist.getX() * cos + twist.getY() * sin;


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


        // telemetry.addData("Forward Offset: ", deltaBackDistance / phi);
        // telemetry.addData("Forward Offset: ", pose.getY() / pose.getHeading());
        // telemetry.addData("Velocity: ", velocity.getHeading());
        // telemetry.addData("Heading Error: ", actualHeading - pose.getHeading());

        // this.telemetry.addData("Perp: ", deltaPerpendicular);
        // this.telemetry.addData("Middle: ", deltaMiddle);
        // this.telemetry.addData("Back Distance: ", deltaBackDistance);
        // this.telemetry.addData("phi: ", phi);


        // this.telemetry.addData("deltaX: ", deltaX);
        // this.telemetry.addData("deltaY: ", deltaY);
        // this.telemetry.addData("deltaTheta: ", phi);


        /*
        this.telemetry.addData("lp: ", deltaLeftDistance);
        this.telemetry.addData("rp: ", deltaRightDistance);
        this.telemetry.addData("bp: ", deltaBackDistance);
         */

        // Log telemetry data
        this.telemetry.addData("Robot X (inches): ", this.pose.getX());
        this.telemetry.addData("Robot Y (inches): ", this.pose.getY());
        this.telemetry.addData("Robot Heading: ", this.pose.getHeading());

        this.telemetry.addData("Velocity x: ", this.velocity.getX());
        this.telemetry.addData("Velocity y: ", this.velocity.getY());


        this.telemetry.addData("Update Time: ", updateTimer.milliseconds());


    }


    public Pose getPose() {
        return pose;
    }
    public Pose getVelocity() {
        return velocity;
    }

    public void setPose(Pose pose, boolean override) {
        this.pose = pose;
        if (override) {
            imu.setHeadingOffset(pose.getHeading());
            imu.enableHeadingOffsetCorrection();
        }
    }
}
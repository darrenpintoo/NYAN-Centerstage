package org.firstinspires.ftc.teamcode.utilities.localizer;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.BaseEncoder;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;

@Config
public class ThreeWheelLocalizer {

    private Telemetry telemetry;

    private BaseEncoder leftParallel;
    private BaseEncoder rightParallel;
    private BaseEncoder backPerpendicular;

    // Constants for wheel positions
    private static final double WHEEL_BASE = 14.5; // Replace with your robot's wheelbase in inches
    private static final double TRACK_WIDTH = 15.0; // Replace with your robot's track width in inches

    // Variables to store previous encoder values for dead wheel calculations
    private double prevLeftParallelTicks;
    private double prevRightParallelTicks;
    private double prevBackPerpendicularTicks;

    // Variables to store robot pose (x, y, heading)
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    public static double trackWidth = -7.44;
    public static double fowardOffset = 1.87;


    public ThreeWheelLocalizer(BaseEncoder lp, BaseEncoder rp, BaseEncoder bp, Telemetry telemetry) {
        this.leftParallel = lp;
        this.rightParallel = rp;
        this.backPerpendicular = bp;
        this.telemetry = telemetry;

        this.prevLeftParallelTicks = lp.getTicks();
        this.prevRightParallelTicks = rp.getTicks();
        this.prevBackPerpendicularTicks = bp.getTicks();
    }
    public void updatePose() {
        double currentLeftParallelTicks = leftParallel.getTicks() * 1.0138;
        double currentRightParallelTicks = rightParallel.getTicks() * 1.0138;
        double currentBackPerpendicularTicks = backPerpendicular.getTicks();

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
        double deltaMiddle = (deltaLeftDistance + deltaRightDistance) / 2.0;
        double deltaPerpendicular = deltaBackDistance - fowardOffset * phi;

        double deltaX = deltaMiddle * Math.cos(robotHeading) - deltaPerpendicular * Math.sin(robotHeading);
        double deltaY = deltaMiddle * Math.sin(robotHeading) + deltaPerpendicular * Math.cos(robotHeading);

        // Update robot pose
        robotX += deltaX;
        robotY += deltaY;
        robotHeading += phi;


        this.telemetry.addData("Forward Offset: ", deltaBackDistance / phi);
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
        this.telemetry.addData("Robot X (inches): ", robotX);
        this.telemetry.addData("Robot Y (inches): ", robotY);
        this.telemetry.addData("Robot Heading: ", robotHeading);


    }


    public Pose getPose() {
        return new Pose(this.robotX, this.robotY, this.robotHeading);
    }
}
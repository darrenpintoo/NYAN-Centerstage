package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionBlueFar;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineBlueFar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
public class FarGateBlueAuto extends LinearOpMode {


    PropDetectionBlueFar propDetectionRed;
    String webcamName = "Webcam 1";

    RobotEx robot = RobotEx.getInstance();
    OneWheelOdometryDrive drive;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private VisionPortal visionPortal2;
    private PropDetectionPipelineBlueFar propDetector;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(
                CameraConstants.fx,
                CameraConstants.fy,
                CameraConstants.cx,
                CameraConstants.cy
        ).build();



        propDetector = new PropDetectionPipelineBlueFar();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(propDetector)
                .enableLiveView(true)
                .build();



        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.update();
        }

        waitForStart();
        robot.intake.disableTeleop();

        PlacementPosition placementPosition = propDetector.getPlacementPosition();

        if (isStopRequested()) return;

        visionPortal2.stopStreaming();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, true);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        robot.postInit();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        ElapsedTime wok = new ElapsedTime();

        robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        // robot.pause(1);
        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);


        double a = DriveConstants.MAX_ACCELERATION;
        double v = DriveConstants.MAX_VELOCITY;

        double pixelOffset = 0;
        double yPixelOffset = 0;
        PIDDrive.aMax = 35;
        PIDDrive.vMax = 50;

        switch (placementPosition) {
            case CENTER:
                drive.gotoPoint(new Pose(-31, 18, Math.PI / 2));
                robot.intake.reset();
                robot.pause(0.75);
                drive.gotoPoint(new Pose(-40, 33, Math.PI / 2));
                drive.gotoPoint(new Pose(-8, 33, Math.PI / 2));
                pixelOffset = -1;
                break;
            case LEFT:
                drive.gotoPoint(new Pose(-32, 15, Math.PI / 2));
                drive.turnToAngle(Math.PI);
                drive.gotoPoint(new Pose(-32, 18, Math.PI));
                robot.intake.reset();
                robot.pause(0.5);
                drive.gotoPoint(new Pose(-32, 27, Math.PI));
                drive.turnToAngle(Math.PI / 2);

                pixelOffset = -2;
                yPixelOffset = 1;
                break;
            case RIGHT:
                robot.pause(1);
                drive.gotoPoint(new Pose(-40, 25, Math.PI / 2));
                robot.intake.reset();
                robot.pause(0.25);
                drive.gotoPoint(new Pose(-50, 12.5, Math.PI / 2));

                break;
        }


        drive.gotoPoint(new Pose(-6.5, 15, Math.PI / 2));
        drive.turnToAngle(0);
        double xOffset = 7;
        PIDDrive.aMax = 15;
        PIDDrive.vMax = 25;
        // drive.gotoPoint(new Pose(-8.5 + xOffset, 30, 0));
        robot.intake.setOffset(2.5);
        drive.gotoPoint(new Pose(-8.5 + pixelOffset + xOffset, 37 + yPixelOffset, 0));
        robot.pause(0.25);

        if (robot.intake.getRightProximity() && robot.intake.getLeftProximity() || robot.intake.getCenterProximity()) {

        } else if (robot.intake.getRightProximity()) {
            robot.localizer.setPose(new Pose(-6.5 + pixelOffset + xOffset, robot.localizer.getPose().getY(), robot.localizer.getPose().getHeading()), false);
            // drive.gotoPoint(new Pose(-10.5 + xOffset, 37, 0));
        } else if (robot.intake.getLeftProximity()) {
            robot.localizer.setPose(new Pose(-10.5 + pixelOffset + xOffset, robot.localizer.getPose().getY(), robot.localizer.getPose().getHeading()), false);

            // drive.gotoPoint(new Pose(-6.5 + xOffset, 37, 0));
        }

        drive.gotoPoint(new Pose(-8.5 + pixelOffset + xOffset, 37 + yPixelOffset, 0));

        robot.pause(0.25);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.5);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        PIDDrive.aMax = 35;
        PIDDrive.vMax = 50;
        drive.gotoPoint(new Pose(-4 + xOffset, -55, 0));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);

        double xCorrection = 0;
        double x = 0;
        double y = 0;
        double rot = 0;
        double targetID = 0;

        if (placementPosition == PlacementPosition.LEFT) {
            targetID = 1;
        } else if (placementPosition == PlacementPosition.CENTER) {
            targetID = 2;
        } else {
            targetID = 3;
        }


        switch (placementPosition) {
            case CENTER:
                drive.gotoPoint(new Pose(-32.5 + xOffset, -60, 0));
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(-26 + xOffset, -60, 0));

                break;
            case LEFT:
                drive.gotoPoint(new Pose(-42 + xOffset, -60, 0));
                break;
        }

        wok.reset();

        /*
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id == targetID) {
                xCorrection = detection.ftcPose.x;
                x = detection.ftcPose.range * Math.cos(Math.toRadians(detection.ftcPose.bearing));
                y = detection.ftcPose.range * Math.sin(Math.toRadians(detection.ftcPose.bearing));

                break;
            }
        }

        while (wok.seconds() < 5) {
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                if (detection.id == targetID) {
                    telemetry.addData("iddd: ", detection.id);
                    xCorrection = detection.ftcPose.x;
                    x = detection.ftcPose.range * Math.cos(Math.toRadians(detection.ftcPose.bearing));
                    y = detection.ftcPose.range * Math.sin(Math.toRadians(detection.ftcPose.bearing));
                    rot = detection.ftcPose.yaw;
                    break;
                }
                telemetry.addData("correction: ", detection.ftcPose.x);


            }
            telemetry.addData("true correction: ", xCorrection);

            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);


            telemetry.addData("id: ", targetID);
            telemetry.addData("fps: ", visionPortal.getFps());
            telemetry.addData("detections: ", aprilTag.getDetections().size());

            robot.update();
        }

         */





        drive.gotoPoint(new Pose(
                robot.localizer.getPose().getX() + xCorrection,
                -73,
                0
        ));

        wok.reset();
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
        robot.pause(0.5);
        while (wok.seconds() < 2) {
            robot.drivetrain.robotCentricDriveFromGamepad(0.2, 0, 0);
            robot.update();
        }

        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.25);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(0.5);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        drive.gotoPoint(new Pose(-20 + xOffset, -68, 0));


        // drive.gotoPoint(new Pose(-8, -30, 0));



        // drive.gotoPoint(new Pose(-57,-70,0));
        // drive.gotoPoint((new Pose(-57,-60,0)));
        // drive.gotoPoint((new Pose(-32,-60,0)));


        /*
        switch (placementPosition) {
            case LEFT:
                drive.gotoPoint(new Pose(-37, -65, 0));
                break;
            case CENTER:
                drive.gotoPoint(new Pose(-31, -65, 0));
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(-27, -65, 0));
                break;
        }

        drive.turnToAngle(0);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
        robot.pause(0.25);


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double xCorrection = 0;

        switch (placementPosition) {
            case LEFT:
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 1) {
                        xCorrection = detection.ftcPose.x;
                        break;
                    }
                }
                drive.gotoPoint(new Pose(-37 + xCorrection, -75, 0));
                break;
            case CENTER:
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 2) {
                        xCorrection = detection.ftcPose.x;
                        break;
                    }
                }
                drive.gotoPoint(new Pose(-31 +xCorrection, -75, 0));
                break;
            case RIGHT:
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 3) {
                        xCorrection = detection.ftcPose.x;
                        break;
                    }
                }
                drive.gotoPoint(new Pose(-27 + xCorrection, -75, 0));
                break;
        }
        wok.reset();
        while (wok.seconds() < 0.5) {
            robot.drivetrain.robotCentricDriveFromGamepad(0.25, 0, 0);
            robot.update();
        }
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(0.1);
        drive.gotoPoint(new Pose(-15,-65,0));
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);

        // drive.gotoPoint(new Pose(-34, 36, 0));
        // drive.gotoPoint(new Pose(-54, 12, 0));

        // double a = DriveConstants.MAX_ACCELERATION;
        // double v = DriveConstants.MAX_VELOCITY;
        // PIDDrive.aMax = 35;
        // PIDDrive.vMax = 50;



        /*
        drive.gotoPoint(new Pose(-9,56,0));
        PIDDrive.aMax = 35;
        PIDDrive.vMax = 50;
        drive.gotoPoint(new Pose(-9,60,0));

         */



        /*

        if (robot.intake.getLeftProximity()) {
            drive.gotoPoint(new Pose(-7.5, 60, 0));
            drive.gotoPoint(new Pose(-7.5, 53, 0));
            robot.intake.reset();
            robot.pause(1.25);
            robot.intake.reset();
            robot.pause(0.5);
            robot.intake.setOffset(2);
            drive.gotoPoint(new Pose(-7.5, 60, 0));

        } else if (robot.intake.getRightProximity()) {
            drive.gotoPoint(new Pose(-10.5, 60, 0));
            drive.gotoPoint(new Pose(-10.5, 53, 0));
            robot.intake.reset();
            robot.pause(1.25);
            robot.intake.reset();
            robot.pause(0.5);
            robot.intake.setOffset(2);
            drive.gotoPoint(new Pose(-10.5, 60, 0));
        } else {
            drive.gotoPoint(new Pose(-9,53,0));
            robot.intake.reset();
            robot.pause(1.25);
            robot.intake.reset();
            robot.pause(0.5);
            robot.intake.setOffset(2);
            drive.gotoPoint(new Pose(-9,60,0));
        }*/



        /*
        robot.intake.setOffset(3);
        drive.gotoPoint(new Pose(-10, -36, 0));
        drive.gotoPoint(new Pose(-9,58,0));
        robot.intake.setOffset(2);
        robot.pause(0.6);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.15);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(-10, -36, 0));
         */




        while (!isStopRequested()) {
            robot.update();
            telemetry.update();
        }
        /*
        drive.gotoPoint(new Pose(4,-36,0));
        robot.intake.setOffset(3);
        drive.gotoPoint(new Pose(11,55,0));
        b = PIDDrive.aMax;
        PIDDrive.aMax = 10;
        drive.gotoPoint(new Pose(9,61,0));
        PIDDrive.aMax = b;
        robot.intake.setOffset(2);
        robot.pause(0.6);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.15);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(4,-36,0));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        drive.gotoPoint(new Pose(32,-44,0));
        wok.reset();

        while (wok.seconds() <= 0.3) {
            robot.depositLift.driveLiftFromGamepad(0.75);
        }

        robot.pause(0.25);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        a = PIDDrive.aMax;
        PIDDrive.aMax = 20;
        drive.gotoPoint(new Pose(32, -52, 0));
        PIDDrive.aMax = a;
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.7);
        wok.reset();
        while (wok.seconds() < 1) {
            robot.depositLift.setBoxState(DepositLift.BoxStates.CLOSED);
            robot.depositLift.setTiltState(DepositLift.TiltStates.DEFAULT);
            robot.update();
        }
        robot.pause(0.5);
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.4);
        drive.gotoPoint(new Pose(32, -45, 0));
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        drive.gotoPoint(new Pose(10, -45, 0));
        robot.pause(2);

         */










        /*
        robot.localizer.setPose(new Pose(10, 10, Math.PI/2));

        drive.gotoPoint(new Pose(20,20,Math.PI/2));

         */


        /*
        robot.localizer.setPose(new Pose(-61.8, -15.6, Math.PI/2));

        drive.gotoPoint(new Pose(-35.5,-48.1,Math.PI/2));



        drive.turnToAngle(0);
        drive.gotoPoint(new Pose(-17,-12,0));
        drive.gotoPoint(new Pose(-17,-33.9,0));
        drive.gotoPoint(new Pose(-17,58.0,0));
        drive.gotoPoint(new Pose(-17,-33.9,0));
        drive.gotoPoint(new Pose(-35.5,-48.1,0));

         */



        /*
        robot.localizer.setPose(new Pose(0, 0, 0));
        drive.gotoPoint(new Pose(10, 10, 0));
         */





    }

    public void alignWithApriltag(int id) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -detection.ftcPose.x*3, Math.toRadians(90));
                break;
            }
        }


    }
}

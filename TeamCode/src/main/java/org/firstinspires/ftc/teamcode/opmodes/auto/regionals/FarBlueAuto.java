package org.firstinspires.ftc.teamcode.opmodes.auto.regionals;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import java.util.List;

@Disabled
public class FarBlueAuto extends LinearOpMode {


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

        visionPortal2.close();
        if (isStopRequested()) return;

        robot.postInit();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDDrive drive = new PIDDrive(robot, this, telemetry);
        OneWheelOdometryDrive time = new OneWheelOdometryDrive(this, telemetry);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .build();


        ElapsedTime wok = new ElapsedTime();

        robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        // robot.pause(1);
        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
        robot.pause(2);


        double a = DriveConstants.MAX_ACCELERATION;
        double v = DriveConstants.MAX_VELOCITY;
        PIDDrive.aMax = 20;
        PIDDrive.vMax = 50;

        switch (placementPosition) {
            case CENTER:
                drive.gotoPoint(new Pose(-30, 15, Math.PI/2));
                robot.intake.reset();
                drive.gotoPoint(new Pose(-40, 15, Math.PI/2));
                drive.gotoPoint(new Pose(-40, 15, 0));
                robot.pause(1);
                drive.gotoPoint(new Pose(-57,15,0));
                break;
            case LEFT:
                drive.gotoPoint(new Pose(-55,19, Math.PI));
                drive.gotoPoint(new Pose(-28, 11, Math.PI));
                robot.intake.reset();
                drive.gotoPoint(new Pose(-28, 19, Math.PI));
                drive.gotoPoint(new Pose(-28, 19, 0));
                robot.pause(1);
                drive.gotoPoint(new Pose(-57,19,0));
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(-55, 25, Math.PI/2));
                drive.gotoPoint(new Pose(-40, 25, Math.PI/2));
                robot.intake.reset();
                drive.gotoPoint(new Pose(-45, 15, Math.PI/2));
                drive.gotoPoint(new Pose(-45, 15, 0));
                robot.pause(1);
                drive.gotoPoint(new Pose(-57,15,0));
                break;
        }
        drive.gotoPoint(new Pose(-57,-70,0));
        drive.gotoPoint((new Pose(-57,-60,0)));
        drive.gotoPoint((new Pose(-32,-60,0)));


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


        visionPortal.close();

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

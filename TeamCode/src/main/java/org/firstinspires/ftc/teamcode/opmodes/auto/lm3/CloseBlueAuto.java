package org.firstinspires.ftc.teamcode.opmodes.auto.lm3;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineRedClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Close Blue Auto")
public class CloseBlueAuto extends LinearOpMode {


    PropDetectionBlueFar propDetectionRed;
    String webcamName = "Webcam 1";

    RobotEx robot = RobotEx.getInstance();
    OneWheelOdometryDrive drive;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private VisionPortal visionPortal2;
    private PropDetectionPipelineBlueClose propDetector;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);







        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(
                CameraConstants.fx,
                CameraConstants.fy,
                CameraConstants.cx,
                CameraConstants.cy
        ).build();



        propDetector = new PropDetectionPipelineBlueClose();

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


        PlacementPosition placementPosition = propDetector.getPlacementPosition();

        robot.postInit();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDDrive drive = new PIDDrive(robot, telemetry);
        OneWheelOdometryDrive time = new OneWheelOdometryDrive(this, telemetry);

        visionPortal2.close();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .build();


        ElapsedTime wok = new ElapsedTime();


        /*
        robot.localizer.setPose(new Pose(10, 10, Math.PI / 2));
        drive.turnToAngle(Math.PI);
        drive.gotoPoint(new Pose(10, 15, Math.PI));*
        */


        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        robot.pause(0.25);
        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
        robot.pause(0.25);
        robot.localizer.setPose(new Pose(-59, -15, Math.PI/2), true);
        switch (placementPosition) {
            case LEFT:
                drive.gotoPoint(new Pose(-37, -40, Math.PI/2));
                drive.turnToAngle(0);
                // drive.gotoPoint(new Pose(40, -53, 0));
                break;
            case CENTER:
                drive.gotoPoint(new Pose(-32, -40, Math.PI/2));
                drive.turnToAngle(0);
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(-28, -40, Math.PI/2));
                drive.turnToAngle(0);
                break;
        }

        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
        robot.pause(0.5);
        wok.reset();
        while (wok.seconds() < 3) {
            robot.drivetrain.robotCentricDriveFromGamepad(0.15, 0, 0);
            robot.update();
        }
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(0.1);


        List<AprilTagDetection> detections = aprilTag.getDetections();

        double b = 0;
        double a = 0;
        switch (placementPosition) {
            case LEFT:
                drive.gotoPoint(new Pose(-30, -32, 0)); //right path
                robot.pause(0.5);
                detections = aprilTag.getDetections();
                wok.reset();
                for (AprilTagDetection detection : detections) {
                    telemetry.addData("y: ", detection.ftcPose.y);
                    telemetry.addData("localizer: ", robot.localizer.getPose().getY());
                    robot.localizer.setPose(new Pose(robot.localizer.getPose().getX(), -55 + detection.ftcPose.y * 0.89, robot.localizer.getPose().getHeading()), false);
                }
                robot.pause(0.5);
                robot.update();
                drive.gotoPoint(new Pose(-30, -32, 0)); //right path
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                drive.gotoPoint(new Pose(-7.5,-36,0));
                robot.intake.setOffset(3);
                drive.gotoPoint(new Pose(-11,55,0));
                b = PIDDrive.aMax;
                PIDDrive.aMax = 10;
                drive.gotoPoint(new Pose(-9,61,0));
                PIDDrive.aMax = b;
                robot.intake.setOffset(2);
                robot.pause(0.6);
                robot.intake.setGripperState(Intake.GripperStates.CLOSED);
                robot.pause(0.15);
                robot.intake.setRotationState(Intake.RotationStates.ROTATED);
                drive.gotoPoint(new Pose(-8,-36,0));
                robot.intake.setGripperState(Intake.GripperStates.OPEN);
                drive.gotoPoint(new Pose(-32,-44,0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                a = PIDDrive.aMax;
                PIDDrive.aMax = 20;
                drive.gotoPoint(new Pose(-32, -52, 0));
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
                drive.gotoPoint(new Pose(-32, -45, 0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                visionPortal.close();

                drive.gotoPoint(new Pose(-10, -45, 0));
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(-30, -10, 0)); //right path
                robot.pause(0.1);
                detections = aprilTag.getDetections();
                wok.reset();
                for (AprilTagDetection detection : detections) {
                    telemetry.addData("y: ", detection.ftcPose.y);
                    telemetry.addData("localizer: ", robot.localizer.getPose().getY());
                    robot.localizer.setPose(new Pose(robot.localizer.getPose().getX(), -55 + detection.ftcPose.y * 0.84, robot.localizer.getPose().getHeading()), false);
                }
                robot.update();

                robot.pause(0.1);
                robot.update();
                drive.gotoPoint(new Pose(-30, -10, 0)); //right path
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                drive.gotoPoint(new Pose(-7.5,-36,0));
                robot.intake.setOffset(3);
                drive.gotoPoint(new Pose(-11,55,0));
                b = PIDDrive.aMax;
                PIDDrive.aMax = 10;
                drive.gotoPoint(new Pose(-9,61,0));
                PIDDrive.aMax = b;
                robot.intake.setOffset(2);
                robot.pause(0.4);
                robot.intake.setGripperState(Intake.GripperStates.CLOSED);
                robot.pause(0.15);
                robot.intake.setRotationState(Intake.RotationStates.ROTATED);
                drive.gotoPoint(new Pose(-7.5,-36,0));
                robot.intake.setGripperState(Intake.GripperStates.OPEN);
                drive.gotoPoint(new Pose(-32,-44,0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                a = PIDDrive.aMax;
                PIDDrive.aMax = 20;
                drive.gotoPoint(new Pose(-32, -52, 0));
                PIDDrive.aMax = a;
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.7);
                wok.reset();
                while (wok.seconds() < 0.7) {
                    robot.depositLift.setBoxState(DepositLift.BoxStates.CLOSED);
                    robot.depositLift.setTiltState(DepositLift.TiltStates.DEFAULT);
                    robot.update();
                }
                robot.pause(0.5);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.4);
                drive.gotoPoint(new Pose(-32, -45, 0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                visionPortal.close();

                drive.gotoPoint(new Pose(-10, -45, 0));
                break;
            case CENTER:
                drive.gotoPoint(new Pose(-22, -20, 0)); //right path
                robot.pause(0.1);
                detections = aprilTag.getDetections();
                wok.reset();
                for (AprilTagDetection detection : detections) {
                    telemetry.addData("y: ", detection.ftcPose.y);
                    telemetry.addData("localizer: ", robot.localizer.getPose().getY());
                    robot.localizer.setPose(new Pose(robot.localizer.getPose().getX(), -55 + detection.ftcPose.y * 0.86, robot.localizer.getPose().getHeading()), false);
                }
                robot.update();

                robot.pause(0.1);
                robot.update();
                drive.gotoPoint(new Pose(-22, -20, 0)); //right path
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                drive.gotoPoint(new Pose(-7.5,-36,0));
                robot.intake.setOffset(3);
                drive.gotoPoint(new Pose(-11,55,0));
                b = PIDDrive.aMax;
                PIDDrive.aMax = 10;
                drive.gotoPoint(new Pose(-9,63,0));
                PIDDrive.aMax = b;
                robot.intake.setOffset(2);
                robot.pause(0.4);
                robot.intake.setGripperState(Intake.GripperStates.CLOSED);
                robot.pause(0.15);
                robot.intake.setRotationState(Intake.RotationStates.ROTATED);
                drive.gotoPoint(new Pose(-8,-36,0));
                robot.intake.setGripperState(Intake.GripperStates.OPEN);
                drive.gotoPoint(new Pose(-32,-44,0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                a = PIDDrive.aMax;
                PIDDrive.aMax = 20;
                drive.gotoPoint(new Pose(-28, -52, 0));
                PIDDrive.aMax = a;
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.7);
                wok.reset();
                while (wok.seconds() < 0.7) {
                    robot.depositLift.setBoxState(DepositLift.BoxStates.CLOSED);
                    robot.depositLift.setTiltState(DepositLift.TiltStates.DEFAULT);
                    robot.update();
                }
                robot.pause(0.5);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.4);
                drive.gotoPoint(new Pose(-28, -45, 0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                visionPortal.close();
                drive.gotoPoint(new Pose(-10, -45, 0));
                break;

        }

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

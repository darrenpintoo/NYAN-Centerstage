package org.firstinspires.ftc.teamcode.opmodes.auto.state;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionBlueFar;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Close Red Auto")
public class CloseRedAuto extends LinearOpMode {


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
        robot.init(this);







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

        boolean backstage = false;
        boolean preload = false;

        while (opModeInInit()) {

            if (gamepad1.a && gamepad1.b) {
                backstage = true;
            }

            if (gamepad1.y && gamepad1.x) {
                preload = true;
            }
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.addData("Backstage: ", backstage);
            telemetry.addData("Preload: ", preload);
            telemetry.update();
        }

        waitForStart();
        robot.intake.disableTeleop();

        PlacementPosition placementPosition = PlacementPosition.LEFT;// propDetector.getPlacementPosition();

        visionPortal2.stopStreaming();
        if (isStopRequested()) return;

        robot.postInit();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDDrive drive = new PIDDrive(robot, this, telemetry);
        OneWheelOdometryDrive time = new OneWheelOdometryDrive(this, telemetry);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();


        ElapsedTime wok = new ElapsedTime();

        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        // robot.pause(1);
        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
        robot.localizer.setPose(new Pose(59, -15, -Math.PI/2), true);
        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(37, -40, -Math.PI/2));
                break;
            case CENTER:
                drive.gotoPoint(new Pose(32, -40, -Math.PI/2));
                break;
            case LEFT:
                drive.gotoPoint(new Pose(24, -40, -Math.PI/2));
                break;
        }

        drive.turnToAngle(0);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);

        double xOffset = 2;
        double yOffset = -6.5;

        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(37 + xOffset, -50 + yOffset, 0));
                break;
            case CENTER:
                drive.gotoPoint(new Pose(32 + xOffset, -50 + yOffset, 0));
                break;
            case LEFT:
                drive.gotoPoint(new Pose(24 + xOffset, -50 + yOffset, 0));
                break;
        }


        wok.reset();
        while (wok.seconds() < 0.75) {
            robot.drivetrain.robotCentricDriveFromGamepad(0.2, 0, 0);
            robot.update();
        }
        double a = PIDDrive.aMax;
        double v = PIDDrive.vMax;
        PIDDrive.aMax = 30;
        PIDDrive.vMax = 40;
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(0.1);

        if (!preload) {
            switch (placementPosition) {
                case LEFT:
                    drive.gotoPoint(new Pose(30 + xOffset, -11 + yOffset, 0)); //right path
                    robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                    robot.intake.reset();
                    drive.gotoPoint(new Pose(3 + xOffset, -25 + yOffset, 0));
                    break;
                case CENTER:
                    drive.gotoPoint(new Pose(19 + xOffset, -27 + yOffset, 0)); //right path
                    robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                    robot.intake.reset();
                    drive.gotoPoint(new Pose(3 + xOffset, -30 + yOffset, 0));
                    break;
                case RIGHT:
                    drive.gotoPoint(new Pose(22 + xOffset, -32 + yOffset, 0));
                    robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                    robot.intake.reset();
                    drive.gotoPoint(new Pose(3 + xOffset, -36 + yOffset, 0));
                    break;
            }

            robot.intake.setOffset(3);


            drive.gotoPoint(new Pose(6 + xOffset, 57 + yOffset, 0));
            PIDDrive.aMax = 15;
            PIDDrive.vMax = 30;
            drive.gotoPoint(new Pose(6 + xOffset, 60 + yOffset, 0));
            robot.intake.setOffset(2);


            if (robot.intake.getRightProximity()) {
                robot.localizer.setPose(new Pose(8 + xOffset, robot.localizer.getPose().getY(), robot.localizer.getPose().getHeading()), false);


            } else if (robot.intake.getLeftProximity()) {
                robot.localizer.setPose(new Pose(4 + xOffset, robot.localizer.getPose().getY(), robot.localizer.getPose().getHeading()), false);

            }

            drive.gotoPoint(new Pose(6 + xOffset, 60 + yOffset, 0));

            PIDDrive.aMax = a;
            PIDDrive.vMax = v;
            // robot.intake.setOffset(1.5);
            // robot.intake.setOffset(3);
            robot.pause(0.8);
            robot.intake.setGripperState(Intake.GripperStates.CLOSED);
            robot.pause(0.15);
            robot.intake.setRotationState(Intake.RotationStates.ROTATED);
            drive.gotoPoint(new Pose(5 + xOffset, -36 + yOffset, 0));

            if (!backstage) {
                robot.intake.setGripperState(Intake.GripperStates.OPEN);
                drive.gotoPoint(new Pose(30 + xOffset, -44 + yOffset, 0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1_AUTO);
                drive.gotoPoint(new Pose(30 + xOffset, -50 + yOffset, 0));
                wok.reset();
                while (wok.seconds() < 0.5) {
                    robot.drivetrain.robotCentricDriveFromGamepad(0.3, 0, 0);
                    robot.update();
                }
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                robot.pause(1);
                drive.gotoPoint(new Pose(30 + xOffset, -45 + yOffset, 0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                drive.gotoPoint(new Pose(13 + xOffset, -45 + yOffset, 0));
            } else {
                drive.gotoPoint(new Pose(10 + xOffset, -58 + yOffset, 0));
                drive.turnToAngle(-Math.PI);
                robot.intake.reset();
                robot.pause(1);
                robot.intake.reset();
            }



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



        } else {
            switch (placementPosition) {
                case LEFT:
                    drive.gotoPoint(new Pose(30 + xOffset, -11 + yOffset, 0)); //right path
                    robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                    robot.intake.reset();
                    break;
                case CENTER:
                    drive.gotoPoint(new Pose(19 + xOffset, -27 + yOffset, 0)); //right path
                    robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                    robot.intake.reset();
                    break;
                case RIGHT:
                    drive.gotoPoint(new Pose(22 + xOffset, -32 + yOffset, 0));
                    robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                    robot.intake.reset();
                    break;
            }

            drive.gotoPoint(new Pose(58 + xOffset, -50 + yOffset, 0)); //right path

        }
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
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -detection.ftcPose.x*3, -Math.toRadians(90));
                break;
            }
        }


    }
}

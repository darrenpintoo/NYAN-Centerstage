package org.firstinspires.ftc.teamcode.opmodes.auto.lm2;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionRed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name="Preload Red v4b")
@Disabled
public class PreloadRedAuto extends LinearOpMode {


    PropDetectionRed propDetectionRed;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    RobotEx robot = RobotEx.getInstance();

    OneWheelOdometryDrive drive;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionRed();
        camera.setPipeline(propDetectionRed);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        while (!isStarted()) {
            telemetry.addData("ROTATION: ", propDetectionRed.getPlacementPosition());
            telemetry.addData("Red Amount 1: ", propDetectionRed.getRedAmount1());
            telemetry.addData("Red Amount 2: ", propDetectionRed.getRedAmount2());


            telemetry.update();
        }

        waitForStart();


        PlacementPosition placementPosition = propDetectionRed.getPlacementPosition();

        camera.closeCameraDevice();
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        // Notify subsystems before loop
        robot.postInit();
        drive = new OneWheelOdometryDrive(this, telemetry);

        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        robot.intake.onCyclePassed();
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);

        waitForStart();

        /*
        drive.driveForward(robot.drivetrain.rightBackMotor, 20, Math.toRadians(0));
        drive.turnToAngle(Math.toRadians(-90));
        drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(-90));
        drive.turnToAngle(Math.toRadians(0));
        drive.driveForward(robot.drivetrain.rightBackMotor, 30, Math.toRadians(0));
         */

        switch (placementPosition) {
            case LEFT:
                robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
                robot.intake.setOffset(0);
                drive.driveForward(robot.drivetrain.rightBackMotor, 38, Math.toRadians(0));
                break;
            case RIGHT:
                robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
                robot.intake.setOffset(0);
                drive.driveForward(robot.drivetrain.rightBackMotor, 4, Math.toRadians(0));
                drive.strafeRight(robot.drivetrain.rightBackMotor, 19, Math.toRadians(0));
                drive.driveForward(robot.drivetrain.rightBackMotor, 25, Math.toRadians(0));
                break;
            case CENTER:
                robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
                robot.intake.setOffset(0);
                drive.driveForward(robot.drivetrain.rightBackMotor, 38, Math.toRadians(0));
                break;
        }


        switch (placementPosition) {
            case LEFT:
                drive.strafeRight(robot.drivetrain.leftFrontMotor, 10, -Math.toRadians(0));
                drive.turnToAngle(-Math.toRadians(90));
                drive.driveForward(robot.drivetrain.rightBackMotor, 8, -Math.toRadians(90));
                // drive.driveForward(robot.drivetrain.rightBackMotor, -30, Math.toRadians(90));
                break;
            case RIGHT:
                robot.pause(1);
                break;
            case CENTER:
                drive.driveForward(robot.drivetrain.rightBackMotor, -2.5, Math.toRadians(0));
                break;

        }

        robot.pause(0.25);
        robot.intake.reset();
        robot.pause(1);
        robot.intake.setOffset(5);
        robot.pause(0.5);


        switch (placementPosition) {
            case LEFT:
                drive.driveForward(robot.drivetrain.rightBackMotor, -30, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.strafeRight(robot.drivetrain.rightBackMotor,10, Math.toRadians(-90));
                alignWithApriltag(4);
                drive.driveForward(robot.drivetrain.rightBackMotor, -30, Math.toRadians(-90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -45, Math.toRadians(-90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 30, Math.toRadians(-90));
                }
                break;
            case RIGHT:
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(0));
                drive.turnToAngle(Math.toRadians(-90));
                drive.driveForward(robot.drivetrain.rightBackMotor, -25, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.strafeRight(robot.drivetrain.rightBackMotor,  10, Math.toRadians(-90));
                alignWithApriltag(6);
                drive.driveForward(robot.drivetrain.rightBackMotor, -20, Math.toRadians(-90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -20, Math.toRadians(-90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(-90));
                }
                break;
            case CENTER:
                drive.driveForward(robot.drivetrain.rightBackMotor, -4, Math.toRadians(0));
                drive.turnToAngle(Math.toRadians(-90));
                drive.driveForward(robot.drivetrain.rightBackMotor, -25, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.strafeRight(robot.drivetrain.leftFrontMotor, 9, Math.toRadians(-90));
                alignWithApriltag(5);
                drive.driveForward(robot.drivetrain.rightBackMotor, -30, Math.toRadians(-90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -35, Math.toRadians(-90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(-90));
                    // drive.strafeRight(robot.drivetrain.leftFrontMotor, 45, Math.toRadians(-90));
                    // drive.driveForward(robot.drivetrain.rightBackMotor, 100, Math.toRadians(-90));
                }
                break;





/*
        switch (placementPosition) {
            case LEFT:
                drive.driveForward(robot.drivetrain.rightBackMotor, -40, Math.toRadians(90));
                robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
                drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(90));
                drive.strafeRight(robot.drivetrain.leftFrontMotor, 25, Math.toRadians(90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -60, Math.toRadians(90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(90));
                }
                break;
            case RIGHT:
                drive.driveForward(robot.drivetrain.rightBackMotor, -85, Math.toRadians(90));
                robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
                drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(90));
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -4, Math.toRadians(90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 60, Math.toRadians(90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -30, Math.toRadians(90));
                }
                break;
            case CENTER:
                drive.driveForward(robot.drivetrain.rightBackMotor, -16, Math.toRadians(-0));
                drive.turnToAngle(Math.toRadians(90));
                drive.driveForward(robot.drivetrain.rightBackMotor, -70, Math.toRadians(90));
                robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
                drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(90));
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -16, Math.toRadians(90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);

                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -40, Math.toRadians(90));
                }
                break;


        }

 */

        }
    }

    public void alignWithApriltag(int id) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection == null) {
                return;
            }
            if (detection.id == id) {
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -detection.ftcPose.x*3, Math.toRadians(-90));
                break;
            }
        }


    }
}

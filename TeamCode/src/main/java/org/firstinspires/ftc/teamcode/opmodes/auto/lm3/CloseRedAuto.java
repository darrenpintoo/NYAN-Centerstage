package org.firstinspires.ftc.teamcode.opmodes.auto.lm3;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionBlueFar;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Autonomous(name = "Close Red Auto")
public class CloseRedAuto extends LinearOpMode {


    PropDetectionBlueFar propDetectionRed;
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


        /*
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

         */



        waitForStart();


        PlacementPosition placementPosition = PlacementPosition.RIGHT;

        robot.postInit();
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDDrive drive = new PIDDrive(robot, telemetry);
        OneWheelOdometryDrive time = new OneWheelOdometryDrive(this, telemetry);


        /*
        robot.localizer.setPose(new Pose(10, 10, Math.PI / 2));
        drive.turnToAngle(Math.PI);
        drive.gotoPoint(new Pose(10, 15, Math.PI));*
        */


        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        robot.pause(3);
        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);

        robot.localizer.setPose(new Pose(59, -15, -Math.PI/2));
        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(25, -40, -Math.PI/2));
                drive.turnToAngle(0);
                drive.gotoPoint(new Pose(25, -53, 0));
                break;
            case CENTER:
                break;
            case LEFT:
                break;
        }

        robot.pause(0.5);
        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(36, -32, 0)); //right path
                break;
            case LEFT:
                drive.gotoPoint(new Pose(36, -20, 0)); //right path
                break;
            case CENTER:
                break;
        }
        drive.gotoPoint(new Pose(12,-36,0));

        /*drive.gotoPoint(new Pose(12,63,0));
        drive.gotoPoint(new Pose(12,-36,0));
        drive.gotoPoint(new Pose(36,-44,0));
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

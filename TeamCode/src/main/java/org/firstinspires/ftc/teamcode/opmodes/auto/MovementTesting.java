package org.firstinspires.ftc.teamcode.opmodes.auto;

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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Movement Testing")
public class MovementTesting extends LinearOpMode {


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

        while (opModeInInit()) {

            if (gamepad1.a && gamepad1.b) {
                backstage = true;
            }
            telemetry.addLine("ready");
            telemetry.addData("position", propDetector.getPlacementPosition());
            telemetry.addData("1: ", propDetector.getRedAmount1());
            telemetry.addData("2: ", propDetector.getRedAmount2());
            telemetry.addData("Backstage: ", backstage);
            telemetry.update();
        }

        waitForStart();
        robot.intake.disableTeleop();

        PlacementPosition placementPosition = propDetector.getPlacementPosition();

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

        robot.localizer.setPose(new Pose(0, 0, Math.PI / 2), true);

        drive.gotoPoint(new Pose(10, -10, Math.PI / 2));
        drive.turnToAngle(0);
        for (int i = 0; i < 3; i++) {
            drive.gotoPoint(new Pose(15, -20, 0));
            drive.gotoPoint(new Pose(20, -15, 0));
            drive.gotoPoint(new Pose(15, -10, 0));
            drive.gotoPoint(new Pose(10, -13, 0));
        }

        drive.gotoPoint(new Pose(10, -10, 0));
        drive.turnToAngle(Math.toRadians(90));

        drive.gotoPoint(new Pose(6, 0, Math.PI / 2));


        // drive.gotoPoint(new Pose(0, 10, 0));
        // drive.turnToAngle(Math.toRadians(90));
        // drive.gotoPoint(new Pose(0, 10, Math.toRadians(90)));
        // drive.turnToAngle(Math.toRadians(-90));
        // drive.gotoPoint(new Pose(0, 10, Math.toRadians(-90)));
        // drive.turnToAngle(Math.toRadians(0));
        // drive.gotoPoint(new Pose(0, 10, 0));
        // drive.gotoPoint(new Pose(0, 2, 0));

        /*
        for (int i = 0; i < 4; i++) {
            // drive.gotoPoint(new Pose(15, 20, 0));
            // drive.gotoPoint(new Pose(0, 20, 0));



            drive.gotoPoint(new Pose(10, 10, 0));
            drive.gotoPoint(new Pose(0, 10, 0));

            // drive.gotoPoint(new Pose(0, 10, 0));
            // drive.gotoPoint(new Pose(0, 3, 0));
        }

         */
        // drive.gotoPoint(new Pose(15, 25, 0));
        // drive.gotoPoint(new Pose(5, 15, 0));
        // drive.gotoPoint(new Pose(5, 20, 0));
        // drive.turnToAngle(Math.toRadians(90));
        // drive.gotoPoint(new Pose(5, 5, 0));
        // drive.turnToAngle(Math.toRadians(0));
        // drive.gotoPoint(new Pose(0, 3, 0));





        while (!robot.stopRequested) {
            robot.update();
        }
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

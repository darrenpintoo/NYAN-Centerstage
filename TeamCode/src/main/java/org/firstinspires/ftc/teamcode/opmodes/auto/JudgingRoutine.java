package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineRedClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Judging Routine")
public class JudgingRoutine extends LinearOpMode {


    PropDetectionBlueFar propDetectionRed;
    String webcamName = "Webcam 1";

    RobotEx robot = RobotEx.getInstance();
    OneWheelOdometryDrive drive;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private VisionPortal visionPortal2;
    private PropDetectionPipelineRedClose propDetector;

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



        propDetector = new PropDetectionPipelineRedClose();

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

        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);
        robot.intake.setGripperState(Intake.GripperStates.OPEN);

        while (!gamepad1.a) {
            robot.update();
        }

        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        robot.pause(1.5);
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        robot.pause(1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL3);
        robot.pause(1.9);
        robot.depositLift.flutterBox();
        robot.pause(1);
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.5);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);

        while (!isStopRequested()) {
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

package org.firstinspires.ftc.teamcode.opmodes.auto.lm1;

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
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionBlue;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Preload Blue Auto 2+2 TREVOR WINANDY")
@Disabled
public class PreloadBlueAuto extends LinearOpMode {


    PropDetectionBlue propDetectionRed;
    OpenCvCamera camera;
    String webcamName = "Webcam 2";

    RobotEx robot = RobotEx.getInstance();



    @Override
    public void runOpMode() {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionBlue();
        camera.setPipeline(propDetectionRed);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        while (!isStarted()) {
            telemetry.addData("ROTATION: ", propDetectionRed.getPlacementPosition());
            telemetry.addData("Red Amount 1: ", propDetectionRed.getRedAmount1());
            telemetry.addData("Red Amount 2: ", propDetectionRed.getRedAmount2());

            telemetry.update();
        }

        waitForStart();

        PlacementPosition placementPosition = propDetectionRed.getPlacementPosition();

        // Notify subsystems before loop
        robot.postInit();
        OneWheelOdometryDrive drive = new OneWheelOdometryDrive(this, telemetry);

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
        robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
        robot.intake.setOffset(3);
        drive.driveForward(robot.drivetrain.rightBackMotor, 55, Math.toRadians(0));

        switch (placementPosition) {
            case LEFT:
                drive.turnToAngle(Math.toRadians(90));
                drive.driveForward(robot.drivetrain.rightBackMotor, -30, Math.toRadians(90));
                break;
            case RIGHT:
                drive.turnToAngle(Math.toRadians(90));
                drive.driveForward(robot.drivetrain.rightBackMotor, 9, Math.toRadians(90));
                break;
            case CENTER:
                drive.driveForward(robot.drivetrain.rightBackMotor, -2.5, Math.toRadians(0));
                break;

        }
        robot.pause(1);
        robot.intake.setOffset(0);
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        robot.pause(1);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        robot.pause(2);


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
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -4, Math.toRadians(90));
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
                drive.strafeRight(robot.drivetrain.leftFrontMotor, 4, 90);
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
                break;
        }
        robot.depositLift.setBoxState(DepositLift.BoxStates.CLOSED);
        drive.driveForward(robot.drivetrain.rightBackMotor, 114, Math.toRadians(90));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        robot.intake.setOffset(5);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        drive.driveForward(robot.drivetrain.rightBackMotor,-114, Math.toRadians(90));
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(90));
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
            drive.strafeRight(robot.drivetrain.leftFrontMotor, -60, Math.toRadians(90));
        } else {
            drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(90));
        }



    }
}

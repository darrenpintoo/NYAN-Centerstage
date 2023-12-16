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
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled

@Autonomous(name="Preload Red Auto")
public class PreloadRedAuto extends LinearOpMode {


    PropDetectionRed propDetectionRed;
    OpenCvCamera camera;
    String webcamName = "Webcam 2";

    RobotEx robot = RobotEx.getInstance();



    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionRed();
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
        robot.planeLauncher.setLiftState(PlaneLauncher.AirplaneLiftStates.UP);
        robot.planeLauncher.setShootState(PlaneLauncher.AirplaneShootStates.CLOSED);
        robot.planeLauncher.onCyclePassed();
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
            case RIGHT:
                drive.turnToAngle(Math.toRadians(-90));
                drive.driveForward(robot.drivetrain.rightBackMotor, -30, Math.toRadians(-90));
                break;
            case LEFT:
                drive.turnToAngle(Math.toRadians(-90));
                drive.driveForward(robot.drivetrain.rightBackMotor, 9, Math.toRadians(-90));
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
            case RIGHT:
                drive.driveForward(robot.drivetrain.rightBackMotor, -40, Math.toRadians(-90));
                robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
                drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(-90));
                drive.strafeRight(robot.drivetrain.leftFrontMotor, -30, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(-90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -70, Math.toRadians(-90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(-90));
                }
                break;
            case LEFT:
                drive.driveForward(robot.drivetrain.rightBackMotor, -85, Math.toRadians(-90));
                robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
                drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(-90));
                // drive.strafeRight(robot.drivetrain.leftFrontMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(-90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                if (DriveConstants.parkPosition != DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 30, Math.toRadians(-90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -60, Math.toRadians(-90));
                }
                break;
            case CENTER:
                drive.driveForward(robot.drivetrain.rightBackMotor, -16, Math.toRadians(-0));
                drive.turnToAngle(Math.toRadians(-90));
                drive.driveForward(robot.drivetrain.rightBackMotor, -70, Math.toRadians(-90));
                robot.intake.setRotationState(Intake.RotationStates.DEFAULT);
                drive.driveForward(robot.drivetrain.rightBackMotor, 5, Math.toRadians(-90));
                drive.strafeRight(robot.drivetrain.leftFrontMotor, 23, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.driveForward(robot.drivetrain.rightBackMotor, -7, Math.toRadians(-90));
                robot.pause(1);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.driveForward(robot.drivetrain.rightBackMotor, 10, Math.toRadians(-90));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);

                if (DriveConstants.parkPosition == DriveConstants.ParkPositions.LEFT) {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, -50, Math.toRadians(-90));
                } else {
                    drive.strafeRight(robot.drivetrain.leftFrontMotor, 40, Math.toRadians(-90));
                }
                break;


        }
        robot.pause(5);

    }
}

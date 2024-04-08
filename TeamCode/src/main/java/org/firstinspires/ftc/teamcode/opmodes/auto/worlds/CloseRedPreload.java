package org.firstinspires.ftc.teamcode.opmodes.auto.worlds;

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
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionBlueFar;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineRedCloseN;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Close Red Auto Preload", preselectTeleOp = "Main Teleop")
public class CloseRedPreload extends LinearOpMode {



    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        boolean backstage = false;
        boolean preload = false;

        PropDetectionPipelineRedCloseN propPipeline = robot.camera.red;

        robot.camera.backVisionPortal.setProcessorEnabled(propPipeline, true);


        while (opModeInInit()) {

            if (gamepad1.a && gamepad1.b) {
                backstage = true;
            } else if (gamepad1.x && gamepad1.y) {
                preload = true;
            }
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeline.getPlacementPosition());
            telemetry.addData("1: ", propPipeline.getRedAmount1());
            telemetry.addData("2: ", propPipeline.getRedAmount2());
            telemetry.addData("Backstage: ", backstage);
            telemetry.addData("Preload: ", preload);

            telemetry.update();
        }

        waitForStart();

        robot.postInit();

        if (isStopRequested()) return;

        robot.intake.disableTeleop();

        PlacementPosition placementPosition = propPipeline.getPlacementPosition();



        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);

        robot.update();

        robot.localizer.setPose(new Pose(61, -13, Math.PI/2), true);
        robot.pause(0.1);
        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(40, -35, 0), 0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(41.41,-48, 0), 0);
                drive.gotoPoint(new Pose(41.41,-50, 0), 0);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(26, -30, 0), 0);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                break;
            case CENTER:
                drive.gotoPoint(new Pose(36, -35, 0), 0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(35.41,-48, 0), 0);
                drive.gotoPoint(new Pose(35.41,-50, 0), 0);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(23, -24, 0), 0);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                robot.pause(0.1);
                break;
            case LEFT:
                drive.gotoPoint(new Pose(29, -35, 0), 0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(29.41,-48, 0), 0);
                drive.gotoPoint(new Pose(29.41,-50, 0), 0);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(30, -7, 0), 0);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                robot.pause(0.1);

                break;
        }

        drive.gotoPoint(new Pose(60, -40, 0), 0.25);

        drive.gotoPoint(new Pose(60, -56, 0), 0.25);










        while (!isStopRequested()) {
            robot.update();
            telemetry.update();
        }




    }

}

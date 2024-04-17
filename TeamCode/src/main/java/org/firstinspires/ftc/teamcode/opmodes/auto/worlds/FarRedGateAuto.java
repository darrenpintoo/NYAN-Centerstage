package org.firstinspires.ftc.teamcode.opmodes.auto.worlds;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementUtils;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineBlueCloseN;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineBlueFarN;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineRedFarN;

@Autonomous(name = "Far Red Gate 2+3", group = "Red Far", preselectTeleOp = "Main Teleop")
public class FarRedGateAuto extends LinearOpMode {
    RobotEx robot = RobotEx.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        boolean backstage = false;
        boolean preload = false;

        PropDetectionPipelineRedFarN propPipeline = robot.camera.redFar;
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

        robot.camera.preloadPipeline.setTargetAprilTagID(placementPosition.getPosition() + 3);
        robot.localizer.setPose(new Pose(61, 13, Math.PI/2), true);
        robot.pause(0.1);

        switch (placementPosition) {
            case LEFT:
                drive.gotoPoint(new Pose(22, 21.5, Math.PI / 2), -0.25);
                robot.intake.reset();
                robot.pause(0.25);
                drive.gotoPoint(new Pose(11, 21.5, Math.PI / 2), -0.25);
                robot.intake.setOffset(4);
                drive.turnToAngle(0);
                drive.gotoPoint(new Pose(11, 23, 0), 0.25);
                break;
            case CENTER:
                drive.gotoPoint(new Pose(13.5, 14, Math.PI / 2), -0.25);
                robot.intake.reset();
                robot.pause(0.25);
                drive.gotoPoint(new Pose(11, 14, Math.PI / 2), -0.25);
                robot.intake.setOffset(4);
                drive.turnToAngle(0);
                drive.gotoPoint(new Pose(11, 23, 0), 0.25);
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(26, 13, Math.PI / 2), -0.3);
                drive.turnToAngle(3, new MovementConstants(0, 0, -0.5));
                drive.gotoPoint(new Pose(26, 9, 3), -0.25);
                robot.pause(0.25);
                robot.intake.reset();
                robot.pause(0.1);
                drive.gotoPoint(new Pose(26, 15, 3), -0.25);
                drive.turnToAngle(0, new MovementConstants(0, 0, -0.75));
                robot.intake.setOffset(4);
                drive.gotoPoint(new Pose(11, 23,0), 0);

        }

        robot.localizer.setPose(robot.camera.getRobotPoseFromStack(), false);
        robot.intake.setOffset(2.5);
        drive.gotoPoint(new Pose(11, 30, 0), -0.1);
        drive.gotoPoint(new Pose(11, 36, 0), new MovementConstants(10, 10, 0));
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.25);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(11, -60, 0), -0.25);




        robot.intake.setGripperState(Intake.GripperStates.OPEN);

        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(41, -60, 0), 0);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), 0);
                drive.gotoPoint(new Pose(41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                break;
            case CENTER:
                drive.gotoPoint(new Pose(35.41, -60, 0), 0);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(35.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), 0);
                drive.gotoPoint(new Pose(35.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                break;
            case LEFT:
                drive.gotoPoint(new Pose(29.41, -60, 0), 0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(29.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), 0);
                drive.gotoPoint(new Pose(29.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                break;
        }

        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        robot.intake.reset();
        drive.gotoPoint(new Pose(12, -34, 0), -0.25);
        robot.intake.setOffset(1.75);
        drive.gotoPoint(new Pose(12, 53, 0), 0);
        robot.localizer.setPose(robot.camera.getRobotPoseFromStack(), false);
        drive.gotoPoint(new Pose(12, 53, 0), -0.1);
        drive.gotoPoint(new Pose(12, 64, 0), new MovementConstants(10, 50, -0.1));
        robot.pause(0.1);
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.1);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(12, -38, 0), -0.5);
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        drive.gotoPoint(new Pose(29.41, -46, 0), -0.5);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1_AUTO);
        robot.pause(0.25);
        drive.gotoPoint(new Pose(33, -50, 0), new MovementConstants(10, 50, 0));
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(0.75);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        drive.gotoPoint(new Pose(29, -45 ,0), -0.25);
        robot.pause(2);


    }
}

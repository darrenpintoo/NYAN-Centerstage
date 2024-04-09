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

@Autonomous(name = "Close Red Truss 2+2", preselectTeleOp = "Main Teleop")
public class CloseRedTrussAuto extends LinearOpMode {


    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        boolean backstage = false;
        boolean preload = false;

        PropDetectionPipelineBlueCloseN propPipeline = robot.camera.blue;
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

        robot.camera.preloadPipeline.setTargetAprilTagID(placementPosition.getPosition());
        robot.localizer.setPose(new Pose(61, -13, Math.PI/2), true);
        robot.pause(0.1);
        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(38, -35, 0), 0);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), -0.25);
                drive.gotoPoint(new Pose(41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(32, -30, 0), -0.1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                drive.gotoPoint(new Pose(60, -40, 0));

                break;
            case CENTER:
                drive.gotoPoint(new Pose(32, -35, 0), 0);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(35.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), -0.25);
                drive.gotoPoint(new Pose(35.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(25, -24, 0), -0.1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                robot.pause(0.1);
                break;
            case LEFT:
                drive.gotoPoint(new Pose(26, -35, 0), 0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(29.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), -0.25);
                drive.gotoPoint(new Pose(29.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(32, -7, 0), -0.1);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                robot.pause(0.1);
                break;
        }

        drive.gotoPoint(new Pose(60, 57, 0));
        robot.intake.setOffset(3);
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        drive.gotoPoint(new Pose(33, 57, 0));
        robot.pause(0.25);
        robot.camera.waitForFrontCameraFrame();
        Pose correction = robot.camera.stackProcessor.getCorrection();
        drive.gotoPoint(new Pose(33 - correction.getX(), 57, 0));
        robot.intake.setOffset(2);
        drive.gotoPoint(new Pose(33 - correction.getX(), 62, 0), new MovementConstants(10, 10, 0));
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.1);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(41, 57, 0));
        drive.gotoPoint(new Pose(60, 57, 0));
        drive.gotoPoint(new Pose(60, -35, 0));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        drive.gotoPoint(new Pose(42, -35, 0), 0);
        robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
        drive.gotoPoint(new Pose(41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-48, 0), -0.25);
        drive.gotoPoint(new Pose(41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.25);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(1);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        robot.intake.reset();
        drive.gotoPoint(new Pose(60, -35, 0));
        drive.gotoPoint(new Pose(60, -55, 0));






    }
}

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

@Autonomous(name = "Far Blue Truss 2+1", group = "Blue Far", preselectTeleOp = "Main Teleop")
public class FarBlueTrussAuto extends LinearOpMode {
    RobotEx robot = RobotEx.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        boolean backstage = false;
        boolean preload = false;

        PropDetectionPipelineBlueFarN propPipeline = robot.camera.blueFar;
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
        robot.localizer.setPose(new Pose(-61, 13, -Math.PI/2), true);
        robot.pause(0.1);

        switch (placementPosition) {
            case RIGHT:
                drive.gotoPoint(new Pose(-45, 21.5, -Math.PI / 2));
                drive.turnToAngle(Math.PI / 2);
                drive.gotoPoint(new Pose(-34, 21.5, Math.PI / 2));
                robot.intake.reset();
                robot.pause(0.25);
                drive.gotoPoint(new Pose(-37.5, 22, Math.PI / 2));
                drive.turnToAngle(0);
                robot.intake.setOffset(4);
                drive.gotoPoint(new Pose(-36, 24, 0));
                break;
            case CENTER:
                drive.gotoPoint(new Pose(-45, 20, -Math.PI / 2));
                drive.turnToAngle(Math.PI / 2);
                drive.gotoPoint(new Pose(-34, 15, Math.PI / 2));
                robot.intake.reset();
                drive.gotoPoint(new Pose(-36, 22, Math.PI / 2));
                drive.turnToAngle(0);
                robot.intake.setOffset(4);
                drive.gotoPoint(new Pose(-36, 24, 0));
                break;
            case LEFT:
                drive.gotoPoint(new Pose(-30, 20, -Math.PI / 2));
                drive.turnToAngle(-3);
                drive.gotoPoint(new Pose(-30, 9, -3));
                robot.intake.reset();
                drive.gotoPoint(new Pose(-30, 25, -3));
                drive.turnToAngle(0);
                robot.intake.setOffset(4);
                drive.gotoPoint(new Pose(-36, 24, 0));


        }

        robot.localizer.setPose(robot.camera.getRobotPoseFromFrontTags(), false);
        robot.intake.setOffset(2.3);
        drive.gotoPoint(new Pose(-35.125, 60, 0), new MovementConstants(10, 10, 0));
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.5);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(-57, 57, 0), new MovementConstants(50, 25, 0));
        drive.gotoPoint(new Pose(-56, -35,   0), new MovementConstants(50, 10, -0.25));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        MovementUtils.waitForRightClearArea(robot, 4);

        switch (placementPosition) {
            case LEFT:
                drive.gotoPoint(new Pose(-41.41, -35, 0), 0);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(-41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-46, 0), 0);
                drive.gotoPoint(new Pose(-41.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                break;
            case CENTER:
                drive.gotoPoint(new Pose(-35.41, -35, 0), 0);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(-35.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-46, 0), 0);
                drive.gotoPoint(new Pose(-35.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                break;
            case RIGHT:
                drive.gotoPoint(new Pose(-29.41, -35, 0), 0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromBackTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(-29.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-46, 0), 0);
                drive.gotoPoint(new Pose(-29.41 + MovementUtils.getOffsetFromBackdropPlacement(robot),-50, 0), 0);
                break;
        }
        robot.pause(0.5);
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.5);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        robot.pause(0.75);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        robot.intake.reset();
        drive.gotoPoint(new Pose(-58, -40, 0),0);
        drive.gotoPoint(new Pose(-58, -45, 0));

    }
}

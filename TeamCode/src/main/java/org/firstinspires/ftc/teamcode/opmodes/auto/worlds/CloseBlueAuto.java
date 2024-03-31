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
import org.firstinspires.ftc.teamcode.utilities.robot.movement.OneWheelOdometryDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PlacementPosition;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionBlueFar;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PropDetectionPipelineBlueClose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Close Blue Auto 2+4", preselectTeleOp = "Main Teleop")
public class CloseBlueAuto extends LinearOpMode {



    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this);

        boolean backstage = false;
        boolean preload = false;

        PropPipeline propPipeline = robot.camera.propPipeline;

        while (opModeInInit()) {

            if (gamepad1.a && gamepad1.b) {
                backstage = true;
            } else if (gamepad1.x && gamepad1.y) {
                preload = true;
            }
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeline.getLocation());
            telemetry.addData("1: ", propPipeline.getLeftColor());
            telemetry.addData("2: ", propPipeline.getCenterColor());
            telemetry.addData("Backstage: ", backstage);
            telemetry.addData("Preload: ", preload);

            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        robot.intake.disableTeleop();

        PlacementPosition placementPosition = propPipeline.getLocation();


        robot.postInit();

        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);

        robot.intake.setRotationState(Intake.RotationStates.FULL_DEFAULT);

        robot.localizer.setPose(new Pose(-61, -13, -Math.PI/2), true);

        switch (placementPosition) {
            case LEFT:
                drive.gotoPoint(new Pose(-38, -35, 0));
                robot.pause(0.5);
                robot.localizer.setPose(robot.camera.getRobotPoseFromTags(), false);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
                drive.gotoPoint(new Pose(-41.41,-48, 0));
                drive.gotoPoint(new Pose(-41.41,-50, 0));
                robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
                robot.pause(0.25);
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
                drive.gotoPoint(new Pose(-25, -32, 0));
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
                robot.intake.reset();
                drive.gotoPoint(new Pose(-12, -34, 0));
                break;
            case CENTER:
                break;
            case RIGHT:
                break;
        }


        robot.intake.setOffset(2);
        drive.gotoPoint(new Pose(-12, 55, 0));
        drive.gotoPoint(new Pose(-12, 60, 0));
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.1);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(-12, -38, 0));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        drive.gotoPoint(new Pose(-30, -38, 0));
        robot.pause(0.5);
        robot.localizer.setPose(robot.camera.getRobotPoseFromTags(), false);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        drive.gotoPoint(new Pose(-29.41, -48, 0));
        drive.gotoPoint(new Pose(-29.41, -50, 0));
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.25);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
        drive.gotoPoint(new Pose(-12, -34, 0));
        robot.intake.setOffset(1);
        drive.gotoPoint(new Pose(-12, 55, 0));
        drive.gotoPoint(new Pose(-12, 60, 0));
        robot.intake.setGripperState(Intake.GripperStates.CLOSED);
        robot.pause(0.1);
        robot.intake.setRotationState(Intake.RotationStates.ROTATED);
        drive.gotoPoint(new Pose(-12, -38, 0));
        robot.intake.setGripperState(Intake.GripperStates.OPEN);
        drive.gotoPoint(new Pose(-30, -38, 0));
        robot.pause(0.25);
        robot.localizer.setPose(robot.camera.getRobotPoseFromTags(), false);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
        drive.gotoPoint(new Pose(-29.41, -48, 0));
        drive.gotoPoint(new Pose(-29.41, -50, 0));
        robot.depositLift.setBoxState(DepositLift.BoxStates.OPEN);
        robot.pause(0.25);
        robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);








        while (!isStopRequested()) {
            robot.update();
            telemetry.update();
        }




    }
}

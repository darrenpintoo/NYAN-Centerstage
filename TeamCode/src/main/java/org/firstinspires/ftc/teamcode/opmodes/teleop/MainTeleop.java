package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.DepositLift;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.PlaneLauncher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Example teleop code for a basic mecanum drive
 */

@TeleOp(name = "Main Teleop")
public class MainTeleop extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);


        waitForStart();

        // Notify subsystems before loop
        robot.postInit();

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        boolean gripState = false;
        boolean rotationState = false;
        boolean airplaneLaunchState = false;
        boolean airplaneLiftState = false;
        boolean dropState = false;
        // robot.drivetrain.enableAntiTip();

        robot.planeLauncher.setLiftState(PlaneLauncher.AirplaneLiftStates.DOWN);
        robot.update();

        robot.intake.enableTeleop();
        Intake.GripperStates lastGripperState = robot.intake.currentGripperState;
        ElapsedTime e = new ElapsedTime();

        // robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);

        robot.localizer.setPose(new Pose(-61, -13, -Math.PI/2), true);

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        ElapsedTime t = new ElapsedTime();
        while (!robot.stopRequested) {

            e.reset();
            // Retain information about the previous frame's gamepad
            previousFrameGamepad1.copy(currentFrameGamepad1);
            previousFrameGamepad2.copy(currentFrameGamepad2);

            currentFrameGamepad1.copy(gamepad1);
            currentFrameGamepad2.copy(gamepad2);



            // Manual Lift Handler
            {
                if (currentFrameGamepad2.left_trigger > 0) {
                    robot.depositLift.driveLiftFromGamepad(
                            -currentFrameGamepad2.left_trigger
                    );
                } else if (currentFrameGamepad2.right_trigger > 0){
                    robot.depositLift.driveLiftFromGamepad(
                            currentFrameGamepad2.right_trigger
                    );
                }
            }

            // Manual Climb handler
            {
                if (currentFrameGamepad1.left_trigger > 0) {
                    robot.climbLift.setLiftPower(
                            -currentFrameGamepad1.left_trigger
                    );
                } else if (currentFrameGamepad1.right_trigger > 0){
                    robot.climbLift.setLiftPower(
                            currentFrameGamepad1.right_trigger
                    );
                }
            }

            // Manual Intake Grab Handler
            {
                if (currentFrameGamepad1.right_bumper && !previousFrameGamepad1.right_bumper) {
                    gripState = !gripState;

                    robot.intake.setGripperState(
                            robot.intake.currentGripperState == Intake.GripperStates.CLOSED ? Intake.GripperStates.OPEN : Intake.GripperStates.CLOSED
                    );

                    lastGripperState = robot.intake.currentGripperState;

                } else if (lastGripperState != robot.intake.currentGripperState && robot.intake.currentGripperState == Intake.GripperStates.CLOSED) {
                    gamepad1.rumble(250);
                }
            }

            // Manual Intake Rotation Handler
            {
                if (currentFrameGamepad1.left_bumper && !previousFrameGamepad1.left_bumper) {
                    rotationState = !rotationState;
                    robot.intake.setRotationState(
                            robot.intake.currentRotationState != Intake.RotationStates.ROTATED ? Intake.RotationStates.ROTATED : Intake.RotationStates.DEFAULT
                    );
                }
            }

            // Intake Reset
            {
                if (currentFrameGamepad1.x && !previousFrameGamepad1.x) {
                    robot.intake.reset();
                }
            }

            // Manual Plane Launch Handler
            {
                if (currentFrameGamepad2.a && !previousFrameGamepad2.a) {
                    airplaneLaunchState = !airplaneLaunchState;
                    robot.planeLauncher.setShootState(
                            airplaneLaunchState ? PlaneLauncher.AirplaneShootStates.OPENED : PlaneLauncher.AirplaneShootStates.CLOSED
                    );
                }
            }

            // Manual Plane Lifter Handler
            {
                if (currentFrameGamepad2.dpad_right && !previousFrameGamepad2.dpad_right) {
                    airplaneLiftState = !airplaneLiftState;
                    robot.planeLauncher.setLiftState(
                            airplaneLiftState ? PlaneLauncher.AirplaneLiftStates.UP : PlaneLauncher.AirplaneLiftStates.DOWN
                    );
                }
            }

            {
                if (currentFrameGamepad2.y && !previousFrameGamepad2.y) {
                    dropState = !dropState;

                    robot.depositLift.setBoxState(
                            robot.depositLift.boxState == DepositLift.BoxStates.CLOSED ? DepositLift.BoxStates.OPEN : DepositLift.BoxStates.CLOSED
                    );
                }
            }

            {
                if (currentFrameGamepad1.a && !previousFrameGamepad1.a) {
                    robot.intake.incrementOffset(1);
                }

                if (currentFrameGamepad1.b && !previousFrameGamepad1.b) {
                    robot.intake.incrementOffset(-1);
                }
            }

            {
                if (currentFrameGamepad2.right_bumper && !previousFrameGamepad2.right_bumper) {
                    robot.depositLift.incrementOffset(1);
                }

                if (currentFrameGamepad2.left_bumper && !previousFrameGamepad2.left_bumper) {
                    robot.depositLift.incrementOffset(-1);
                }
            }

            if (currentFrameGamepad2.b && !previousFrameGamepad2.b) {
                robot.depositLift.flutterBox();
            }

            if (currentFrameGamepad1.dpad_up && !previousFrameGamepad1.dpad_up) {
                robot.intake.setOffset(2);
            }

            if (currentFrameGamepad1.dpad_left && !previousFrameGamepad1.dpad_left) {
                robot.intake.setOffset(1);
            }

            if (currentFrameGamepad1.dpad_down && !previousFrameGamepad1.dpad_down) {
                robot.intake.setOffset(0);
            }

            {
                if (currentFrameGamepad2.left_stick_button) {
                    robot.depositLift.setBoxState(DepositLift.BoxStates.CLOSED);
                    robot.depositLift.setTiltState(DepositLift.TiltStates.DEFAULT);
                }
            }

            if (currentFrameGamepad2.right_stick_button && previousFrameGamepad2.right_stick_button) {
                robot.depositLift.reset();
                robot.depositLift.setOffset(0);
            }

            robot.drivetrain.robotCentricDriveFromGamepad(
                    currentFrameGamepad1.left_stick_y,
                    currentFrameGamepad1.left_stick_x,
                    currentFrameGamepad1.right_stick_x
            );

            if (currentFrameGamepad2.x && !previousFrameGamepad2.x) {
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL0);
            } else if (currentFrameGamepad2.dpad_down && !previousFrameGamepad2.dpad_down) {
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL1);
            } else if (currentFrameGamepad2.dpad_left && !previousFrameGamepad2.dpad_left) {
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL2);
            } else if (currentFrameGamepad2.dpad_up && !previousFrameGamepad2.dpad_up) {
                robot.depositLift.setTargetState(DepositLift.LiftStates.LEVEL3);
            }




            if (gamepad1.left_stick_button) {
                drive.gotoPoint(new Pose(0, 0, 0));
            }

            if (currentFrameGamepad1.right_stick_button && !previousFrameGamepad1.right_stick_button) {
                robot.localizer.setPose(new Pose(0, 0, 0), true );
            }


            for (AprilTagDetection detection : robot.camera.aprilTagProcessor.getDetections()) {
                telemetry.addData("id: ", detection.id);

                /*
                if (detection.ftcPose == null) {
                    continue;
                }

                double x = detection.ftcPose.x;
                double y = detection.ftcPose.y;

                double rotatedHeading = -robot.localizer.getPose().getHeading();

                double x2 = x * Math.cos(rotatedHeading) + y * Math.sin(rotatedHeading);
                double y2 = x * -Math.sin(rotatedHeading) + y * Math.cos(rotatedHeading);

                telemetry.addData("x trig: ", x2);
                telemetry.addData("y trig: ", y2);

                Pose a = AprilTagLocalization.getRobotPositionFromTag(detection, robot.localizer.getPose().getHeading(), robot.camera.backCameraPose);

                telemetry.addData("Global x: ", a.getX());
                telemetry.addData("Global y: ", a.getY());


                 */

            }



            if (currentFrameGamepad1.y && !previousFrameGamepad1.y) {
                Pose target = AprilTagLocalization.getTagPosition(3);
                target.add(new Pose(0, 20, 0));
                drive.gotoPoint(target);
            }

            if (currentFrameGamepad1.x && !previousFrameGamepad1.x) {
                robot.localizer.setPose(
                        robot.camera.getRobotPoseFromTags(), false
                );
            }








            lastGripperState = robot.intake.currentGripperState;

            double frameTime = robot.update();
            // telemetry.addData("Frame Time: ", MathHelper.truncate(frameTime, 3));
            telemetry.addData("Turn: ", robot.internalIMU.getCurrentFrameHeadingCW());
            // telemetry.addData("Ratio: ", robot.internalIMU.getCurrentFrameHeadingCW()/robot.localizer.getPose().getHeading());
            // telemetry.addData("Effective Track Width: ", ThreeWheelLocalizer.trackWidth / (robot.internalIMU.getCurrentFrameHeadingCW()/robot.localizer.getPose().getHeading()));
        }
    }
}

package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.utilities.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementUtils;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation.StackPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineBlueCloseN;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineBlueFarN;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineRedCloseN;
import org.firstinspires.ftc.teamcode.vision.simulatortests.prop.PropDetectionPipelineRedFarN;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Camera implements Subsystem {

    public AprilTagProcessor backAprilTagProcessor;
    public AprilTagProcessor frontAprilTagProcessor;
    public StackPipeline stackProcessor;
    public PreloadDetectionPipeline preloadPipeline;
    public PropPipeline propPipeline;
    public PropDetectionPipelineBlueCloseN blueClose;
    public PropDetectionPipelineRedCloseN redClose;
    public PropDetectionPipelineBlueFarN blueFar;

    public PropDetectionPipelineRedFarN redFar;



    public VisionPortal backVisionPortal;
    public VisionPortal frontVisionPortal;


    WebcamName backCameraObject;
    WebcamName frontCameraObject;

    boolean setFrontProperties = false;
    boolean setBackProperties = false;

    Telemetry telemetry;

    ArrayList<AprilTagDetection> backDetections = new ArrayList<>();
    ArrayList<AprilTagDetection> frontDetections = new ArrayList<>();
    public MovementUtils.BackdropPosition backdropPosition = MovementUtils.BackdropPosition.LEFT;
    int backDetectionAmount = 0;
    int frontDetectionAmount = 0;
    public Pose backCameraPose = new Pose(-8, 0, 0);
    public Pose frontCameraPose = new Pose(0.5, 0, 0);

    private TwoWheelLocalizer localizer;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        backCameraObject = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontCameraObject = hardwareMap.get(WebcamName.class, "Webcam 2");

        backAprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(
                CameraConstants.BackCamera.fx,
                CameraConstants.BackCamera.fy,
                CameraConstants.BackCamera.cx,
                CameraConstants.BackCamera.cy
        ).setDrawAxes(true).setDrawTagOutline(true).setDrawCubeProjection(true).build();


        frontAprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(
                CameraConstants.FrontCamera.fx,
                CameraConstants.FrontCamera.fy,
                CameraConstants.FrontCamera.cx,
                CameraConstants.FrontCamera.cy
        ).setDrawAxes(true).setDrawTagOutline(true).setDrawCubeProjection(true).build();

        stackProcessor = new StackPipeline();
        preloadPipeline = new PreloadDetectionPipeline();
        propPipeline = new PropPipeline();

        redClose = new PropDetectionPipelineRedCloseN();
        blueClose = new PropDetectionPipelineBlueCloseN();
        blueFar = new PropDetectionPipelineBlueFarN();
        redFar = new PropDetectionPipelineRedFarN();




        frontVisionPortal = new VisionPortal.Builder()
                .setCamera(frontCameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .addProcessors(stackProcessor, frontAprilTagProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        backVisionPortal = new VisionPortal.Builder()
                .setCamera(backCameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .addProcessors(backAprilTagProcessor, preloadPipeline, propPipeline, blueClose, redClose, blueFar, redFar)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        backVisionPortal.setProcessorEnabled(preloadPipeline, false);
        backVisionPortal.setProcessorEnabled(backAprilTagProcessor, false);
        backVisionPortal.setProcessorEnabled(propPipeline, false);
        backVisionPortal.setProcessorEnabled(redClose, false);
        backVisionPortal.setProcessorEnabled(blueClose, false);
        backVisionPortal.setProcessorEnabled(blueFar, false);
        backVisionPortal.setProcessorEnabled(redFar, false);



        frontVisionPortal.setProcessorEnabled(frontAprilTagProcessor, false);
        frontVisionPortal.setProcessorEnabled(stackProcessor, false);


        FtcDashboard.getInstance().startCameraStream(backVisionPortal, 0);

        this.telemetry = telemetry;

        setFrontProperties = false;
        setBackProperties = false;
    }

    @Override
    public void onOpmodeStarted() {
        setBackCameraProperties();
        setFrontCameraProperties();
        localizer = RobotEx.getInstance().localizer;

        backVisionPortal.setProcessorEnabled(preloadPipeline, true);
        backVisionPortal.setProcessorEnabled(backAprilTagProcessor, true);
        backVisionPortal.setProcessorEnabled(propPipeline, false);
        backVisionPortal.setProcessorEnabled(redClose, false);
        backVisionPortal.setProcessorEnabled(blueClose, false);
        backVisionPortal.setProcessorEnabled(blueFar, false);
        backVisionPortal.setProcessorEnabled(redFar, false);

        frontVisionPortal.setProcessorEnabled(frontAprilTagProcessor, true);
        frontVisionPortal.setProcessorEnabled(stackProcessor, true);



    }

    @Override
    public void onCyclePassed() {
        if (!setFrontProperties) {
            setFrontCameraProperties();
        }

        if (!setBackProperties) {
            setBackCameraProperties();
        }

        telemetry.addData("Front fps: ", frontVisionPortal.getFps());
        telemetry.addData("Back fps: ", backVisionPortal.getFps());

        telemetry.addData("front detections: ", frontDetectionAmount);

        telemetry.addData("Left: ", preloadPipeline.leftAverage);
        telemetry.addData("Right: ", preloadPipeline.rightAverage);

        telemetry.addData("Right: ", preloadPipeline.targetAprilTagID);

        telemetry.addData("Vision processor active: ", backVisionPortal.getProcessorEnabled(preloadPipeline));


        if (isFrontCameraActive()) {
            telemetry.addData("Strafe: ", stackProcessor.getStrafeError());
        }

        backDetections = backAprilTagProcessor.getDetections();
        frontDetections = frontAprilTagProcessor.getDetections();

        backDetectionAmount = backDetections.size();
        frontDetectionAmount = frontDetections.size();

        for (AprilTagDetection detection : backDetections) {
            if (detection.ftcPose == null) {
                backDetections.remove(detection);
            }
        }

        for (AprilTagDetection detection : frontDetections) {
            if (detection.ftcPose == null) {
                frontDetections.remove(detection);
            }
        }
    }

    private void setFrontCameraProperties() {
        if (!isFrontCameraActive()) return;

        setFrontProperties = true;

        setCameraProperties(
                frontVisionPortal,
                CameraConstants.FrontCamera.exposure,
                CameraConstants.FrontCamera.gain
        );
    }

    private void setBackCameraProperties() {
        if (!isBackCameraActive()) return;

        setBackProperties = true;

        setCameraProperties(
                backVisionPortal,
                CameraConstants.BackCamera.exposure,
                CameraConstants.BackCamera.gain
        );
    }
    private void setCameraProperties(VisionPortal portal, long exposure, int gain) {

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);

        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }



        exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
        gainControl.setGain(gain);
    }

    public boolean isFrontCameraActive() {
        return frontVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public boolean isBackCameraActive() {
        return backVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public ArrayList<AprilTagDetection> getBackDetections() {
        return backDetections;
    }

    public Pose getRobotPoseFromBackTags() {
        this.waitForBackCameraFrame();

        Pose estimate = new Pose();
        Pose currentPose = localizer.getPose();

        if (backDetectionAmount == 0) return currentPose;

        double heading = localizer.getPose().getHeading();

        for (AprilTagDetection detection : backDetections) {
            estimate.add(AprilTagLocalization.getRobotPositionFromBackTag(detection, heading, backCameraPose));
        }

        estimate.times( 1 / (double) backDetectionAmount);
        estimate.setHeading(heading);

        backdropPosition = preloadPipeline.backdropPosition;

        return estimate;
    }

    public Pose getRobotPoseFromFrontTags() {
        this.waitForFrontCameraFrame();

        Pose estimate = new Pose();
        Pose currentPose = localizer.getPose();

        if (frontDetectionAmount == 0) return currentPose;

        double heading = localizer.getPose().getHeading();

        for (AprilTagDetection detection : frontDetections) {
            estimate.add(AprilTagLocalization.getRobotPositionFromFrontTag(detection, heading, frontCameraPose));
        }

        estimate.times( 1 / (double) frontDetectionAmount);
        estimate.setHeading(heading);

        backdropPosition = preloadPipeline.backdropPosition;

        return estimate;
    }
    public Pose getRobotPoseFromStack() {
        this.waitForFrontCameraFrame();
        Pose correction = stackProcessor.getCorrection();
        correction.add(localizer.getPose());

        return correction;
    }

    public void waitForBackCameraFrame() {
        double fps = backVisionPortal.getFps();
        double exposure = CameraConstants.BackCamera.exposure;

        if (fps == 0) return;

        RobotEx.getInstance().pause( 3 / fps + exposure / 1000 + 0.1);
    }

    public void waitForFrontCameraFrame() {
        double fps = frontVisionPortal.getFps();
        double exposure = CameraConstants.FrontCamera.exposure;


        if (fps == 0) return;

        RobotEx.getInstance().pause(2 / fps + exposure / 1000 + 0.25);
    }
}

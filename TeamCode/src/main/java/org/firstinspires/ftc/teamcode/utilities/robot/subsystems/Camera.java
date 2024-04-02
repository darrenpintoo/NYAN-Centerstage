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
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.distanceestimation.StackPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Camera implements Subsystem {

    public AprilTagProcessor aprilTagProcessor;
    public StackPipeline stackProcessor;
    public PreloadDetectionPipeline preloadPipeline;
    public PropPipeline propPipeline;


    public VisionPortal backVisionPortal;
    public VisionPortal frontVisionPortal;


    WebcamName backCameraObject;
    WebcamName frontCameraObject;

    boolean setFrontProperties = false;
    boolean setBackProperties = false;

    Telemetry telemetry;

    ArrayList<AprilTagDetection> detections = new ArrayList<>();
    int detectionAmount = 0;
    public Pose backCameraPose = new Pose(-8, 0, 0);

    private TwoWheelLocalizer localizer;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        backCameraObject = hardwareMap.get(WebcamName.class, "Webcam 1");
        frontCameraObject = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(
                CameraConstants.BackCamera.fx,
                CameraConstants.BackCamera.fy,
                CameraConstants.BackCamera.cx,
                CameraConstants.BackCamera.cy
        ).setDrawAxes(true).setDrawTagOutline(true).setDrawCubeProjection(true).build();


        stackProcessor = new StackPipeline();
        preloadPipeline = new PreloadDetectionPipeline();
        propPipeline = new PropPipeline();

        frontVisionPortal = new VisionPortal.Builder()
                .setCamera(frontCameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .addProcessor(stackProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        backVisionPortal = new VisionPortal.Builder()
                .setCamera(backCameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .addProcessors(aprilTagProcessor, preloadPipeline, propPipeline)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        backVisionPortal.setProcessorEnabled(preloadPipeline, false);
        backVisionPortal.setProcessorEnabled(aprilTagProcessor, false);
        backVisionPortal.setProcessorEnabled(propPipeline, true);
        FtcDashboard.getInstance().startCameraStream(frontVisionPortal, 0);

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
        backVisionPortal.setProcessorEnabled(aprilTagProcessor, true);
        backVisionPortal.setProcessorEnabled(propPipeline, false);

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

        // telemetry.addData("Left: ", preloadPipeline.leftAverage);
        // telemetry.addData("Right: ", preloadPipeline.rightAverage);

        if (isFrontCameraActive()) {
            telemetry.addData("Strafe: ", stackProcessor.getStrafeError());
        }

        detections = aprilTagProcessor.getDetections();

        detectionAmount = detections.size();

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose == null) {
                detections.remove(detection);
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

    public ArrayList<AprilTagDetection> getDetections() {
        return detections;
    }

    public Pose getRobotPoseFromTags() {
        this.waitForBackCameraFrame();

        Pose estimate = new Pose();
        Pose currentPose = localizer.getPose();

        if (detectionAmount == 0) return currentPose;

        double heading = localizer.getPose().getHeading();

        for (AprilTagDetection detection : detections) {
            estimate.add(AprilTagLocalization.getRobotPositionFromTag(detection, heading, backCameraPose));
        }

        estimate.times( 1 / (double) detectionAmount);
        estimate.setHeading(heading);
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

        RobotEx.getInstance().pause( 2 / fps + exposure / 1000);
    }

    public void waitForFrontCameraFrame() {
        double fps = frontVisionPortal.getFps();
        double exposure = CameraConstants.FrontCamera.exposure;


        if (fps == 0) return;

        RobotEx.getInstance().pause(2 / fps + exposure / 1000);
    }
}

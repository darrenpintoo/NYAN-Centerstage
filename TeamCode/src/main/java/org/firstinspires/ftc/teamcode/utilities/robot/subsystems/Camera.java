package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.utilities.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utilities.math.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.teamcode.vision.simulatortests.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.StackPipeline;
import org.firstinspires.ftc.teamcode.vision.simulatortests.StackProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Camera implements Subsystem {

    public AprilTagProcessor aprilTagProcessor;
    public StackPipeline stackProcessor;
    public PreloadDetectionPipeline preloadPipeline;

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


        stackProcessor = new StackPipeline(telemetry);
        preloadPipeline = new PreloadDetectionPipeline();

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
                .addProcessors(aprilTagProcessor, preloadPipeline)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(backVisionPortal, 0);

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {
        setBackCameraProperties();
        setFrontCameraProperties();

        localizer = RobotEx.getInstance().localizer;
    }

    @Override
    public void onCyclePassed() {
        if (!setFrontProperties) {
            setFrontCameraProperties();
        }

        if (!setBackProperties) {
            setBackCameraProperties();
        }

        telemetry.addData("Front Camera Active: ", isFrontCameraActive());
        telemetry.addData("Back Camera Active: ", isBackCameraActive());

        if (isFrontCameraActive()) {
            telemetry.addData("Strafe: ", stackProcessor.getCorrection().getX());
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
}

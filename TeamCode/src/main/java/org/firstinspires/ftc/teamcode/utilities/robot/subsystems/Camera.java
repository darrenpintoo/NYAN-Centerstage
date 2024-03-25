package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Camera implements Subsystem {

    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal backVisionPortal;
    public VisionPortal frontVisionPortal;


    WebcamName backCameraObject;
    WebcamName frontCameraObject;

    boolean setFrontProperties = false;
    boolean setBackProperties = false;

    Telemetry telemetry;

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


        frontVisionPortal = new VisionPortal.Builder()
                .setCamera(frontCameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        backVisionPortal = new VisionPortal.Builder()
                .setCamera(backCameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(frontVisionPortal, 0);

        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {
        setBackCameraProperties();
        setFrontCameraProperties();
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
}

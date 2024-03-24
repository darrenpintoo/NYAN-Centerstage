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


        if (backVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (backVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = backVisionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = backVisionPortal.getCameraControl(GainControl.class);

        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }


        exposureControl.setExposure(CameraConstants.BackCamera.exposure, TimeUnit.MILLISECONDS);
        gainControl.setGain(CameraConstants.BackCamera.gain);

        if (frontVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (frontVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        exposureControl = frontVisionPortal.getCameraControl(ExposureControl.class);
        gainControl = frontVisionPortal.getCameraControl(GainControl.class);

        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }


        exposureControl.setExposure(CameraConstants.FrontCamera.exposure, TimeUnit.MILLISECONDS);
        gainControl.setGain(CameraConstants.FrontCamera.gain);

    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

    }
}

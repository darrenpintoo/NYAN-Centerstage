package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.vision.simulatortests.CameraConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class BackCamera implements Subsystem {

    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal backVisionPortal;

    WebcamName cameraObject;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        cameraObject = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(
                CameraConstants.BackCamera.fx,
                CameraConstants.BackCamera.fy,
                CameraConstants.BackCamera.cx,
                CameraConstants.BackCamera.cy
        ).setDrawAxes(true).setDrawTagOutline(true).setDrawCubeProjection(true).build();

        backVisionPortal = new VisionPortal.Builder()
                .setCamera(cameraObject)
                .setCameraResolution(new Size(CameraConstants.BackCamera.WIDTH, CameraConstants.BackCamera.HEIGHT))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(backVisionPortal, 0);

        /*
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
        
         */
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

    }
}

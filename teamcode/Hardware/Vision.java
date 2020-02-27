package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Vision extends Mechanism {
    OpenCvCamera camera;
    SkystoneDetector pipeline;

    public Vision() {
    }

    public Vision(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();

        pipeline = new SkystoneDetector();

        camera.setPipeline(pipeline);
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    public String getSkystonePosition() {
        SkystoneDetector.SkystonePosition position = pipeline.getSkystonePosition();
        switch (position) {
            case LEFT_STONE:
                return "left";
            case CENTER_STONE:
                return "center";
            case RIGHT_STONE:
                return "right";
            default:
                return null;
        }
    }
}

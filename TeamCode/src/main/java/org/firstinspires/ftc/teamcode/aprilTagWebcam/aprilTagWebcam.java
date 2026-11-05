package org.firstinspires.ftc.teamcode.aprilTagWebcam;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class aprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // SDK 12+: id, metadata, and center live on AprilTagSingleDetection, not the
    // AprilTagDetection base class, so we keep only single-tag detections here.
    private List<AprilTagSingleDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    // Basic init - no live view (safe for multiple cameras)
    public void init(HardwareMap hwMap, Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, name))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(false)  // Disabled for multi-camera support
                .build();
    }

    // Init with live view container ID (for multiple cameras with preview)
    public void init(HardwareMap hwMap, Telemetry telemetry, String name, int liveViewContainerId) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, name))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .setLiveViewContainerId(liveViewContainerId)
                .build();
    }

    public void update() {
        List<AprilTagSingleDetection> singles = new ArrayList<>();
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                singles.add((AprilTagSingleDetection) detection);
            }
        }
        detectedTags = singles;
    }

    public List<AprilTagSingleDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagSingleDetection detectedId) {
        if (detectedId == null) {
            return;
        }
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagSingleDetection getTagBySpecificId(int id) {
        for (AprilTagSingleDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
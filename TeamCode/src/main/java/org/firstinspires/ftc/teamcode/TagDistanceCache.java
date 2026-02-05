package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TagDistanceCache {

    private final AprilTagProcessor aprilTag;

    // Your scaling: ftcPose.range * 100
    private double lastGoodDistance = -1;
    private long lastSeenTimeMs = 0;

    // How long we trust the last seen distance
    private final long staleAfterMs;

    public TagDistanceCache(AprilTagProcessor aprilTag, long staleAfterMs) {
        this.aprilTag = aprilTag;
        this.staleAfterMs = staleAfterMs;
    }

    /** Call this frequently (every loop) to update cached distance. */
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return;

        // Choose closest tag (lowest range)
        double bestRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : detections) {
            if (d == null || d.ftcPose == null) continue;
            if (d.ftcPose.range > 0 && d.ftcPose.range < bestRange) {
                bestRange = d.ftcPose.range;
            }
        }

        if (bestRange != Double.POSITIVE_INFINITY) {
            lastGoodDistance = bestRange * 100.0; // keep your units
            lastSeenTimeMs = System.currentTimeMillis();
        }
    }

    /**
     * Returns a "stable" distance:
     * - last good distance if a tag was seen recently
     * - otherwise returns -1 (meaning unreliable/not seen)
     */
    public double getStableDistance() {
        long age = System.currentTimeMillis() - lastSeenTimeMs;
        if (lastGoodDistance > 0 && age <= staleAfterMs) return lastGoodDistance;
        return -1;
    }

    /** For telemetry/debugging. */
    public double getLastGoodDistance() {
        return lastGoodDistance;
    }

    /** Age in ms since we last saw a valid tag. */
    public long getAgeMs() {
        return System.currentTimeMillis() - lastSeenTimeMs;
    }

    /** Force-clear cache if you want (optional). */
    public void reset() {
        lastGoodDistance = -1;
        lastSeenTimeMs = 0;
    }
}

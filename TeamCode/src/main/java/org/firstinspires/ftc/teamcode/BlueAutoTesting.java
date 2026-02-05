package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Disabled

@Autonomous(name = "BlueAutoTesting", group = "Robot")
public class BlueAutoTesting extends LinearOpMode {

    // =======================
    // Shooter (Velocity)
    // =======================
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 5200;
    private static final double RPM_TOLERANCE = 200;

    private static final double CLOSE_DIST = 3000;
    private static final double MID_DIST   = 3900;
    private static final double FAR_DIST   = 5500;

    private static final double CLOSE_PWR = 0.33;
    private static final double MID_PWR   = 0.43;
    private static final double FAR_PWR   = 0.53;

    private static final double MAX_DIST_FORCE = 8000;

    private static final long SPINUP_TIMEOUT_MS = 1200;
    private static final long SHOOT_WINDOW_MS   = 4000;
    private static final long CLEAR_AFTER_MS    = 250;

    private static final double FEED_ON_TIME  = 1;  // seconds
    private static final double FEED_OFF_TIME = 0;  // seconds

    private double lastBaseScalar = MID_PWR;

    // =======================
    // Hardware
    // =======================
    private DcMotorEx launcher;
    private DcMotor intake, feeder;

    // =======================
    // Vision
    // =======================
    private VisionPortal portal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // ---- Motors ----
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        feeder = hardwareMap.get(DcMotor.class, "intakeMotor2");

        // ---- Camera / AprilTag ----
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logicam"))
                .addProcessor(aprilTag)
                .build();

        TagDistanceCache tagCache = new TagDistanceCache(aprilTag, 300, 0.25);

        // ---- Road Runner Start ----
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // =======================
        // BUILD YOUR AUTO PATH (EDIT NUMBERS HERE
        Action auto = drive.actionBuilder(startPose)
                .lineToX(-47)                          // back
                //.turn(Math.toRadians(165))             // rotate
                //.strafeTo(new Vector2d(-47, 30))       // strafe (Y coord)
                //.turn(Math.toRadians(-165))            // rotate back
                //.lineToX(-47 + 23)                     // forward (intake move)
                //.lineToX(-47)                          // back again
                .build();

        Action move = drive.actionBuilder(startPose)
                        .turn(90)
                                .build();


        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            safeStopAll();
            if (portal != null) portal.close();
            return;
        }

        // OPTIONAL: shoot before driving


        // ---- RUN ACTION (YOUR RR VERSION NEEDS PACKET) ----
        runActionBlocking(drive, auto);

        // OPTIONAL: shoot after driving
        shootUsingVisionDistance(tagCache);

        runActionBlocking(drive, move);
        safeStopAll();
        if (portal != null) portal.close();

        telemetry.addLine("AUTO DONE");
        telemetry.update();
        sleep(250);
    }

    // =========================
    // Action runner (packet-based)
    // =========================
    private void runActionBlocking(MecanumDrive drive, Action action) {
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean running = action.run(packet);
            drive.updatePoseEstimate();
            if (!running) break;
            idle();
        }
    }

    // =========================
    // Intake helpers
    // =========================
    private void startIntake() {
        intake.setPower(1.0);
        feeder.setPower(0.0);
    }

    private void stopIntake() {
        intake.setPower(0.0);
        feeder.setPower(0.0);
    }

    // =========================
    // SHOOT ROUTINE
    // =========================
    private void shootUsingVisionDistance(TagDistanceCache tagCache) {

        // Stabilize distance reads
        for (int i = 0; i < 10; i++) {
            tagCache.update();
            sleep(20);
        }

        double d = tagCache.getStableDistance(); // -1 if stale/unavailable
        double baseScalar = computeBaseScalar(d);
        if (d >= 0) lastBaseScalar = baseScalar;

        double targetRPM = baseScalar * MAX_RPM;

        launcher.setVelocity(targetRPM * TICKS_PER_REV / 60.0);

        // Spinup time (simple)
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < SPINUP_TIMEOUT_MS) {
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;
            boolean atSpeed = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;

            telemetry.addData("RPM", currentRPM);
            telemetry.addData("Target", targetRPM);
            telemetry.addData("AtSpeed", atSpeed);
            telemetry.addData("Distance", d);
            telemetry.update();

            if (atSpeed) break;
            idle();
        }

        // Feed window
        long feedStart = System.currentTimeMillis();
        boolean feedOn = false;
        long pulseStart = System.currentTimeMillis();

        while (opModeIsActive() && System.currentTimeMillis() - feedStart < SHOOT_WINDOW_MS) {
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;
            boolean atSpeed = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;

            double pulseSec = (System.currentTimeMillis() - pulseStart) / 1000.0;

            if (feedOn && pulseSec >= FEED_ON_TIME) {
                feedOn = false;
                pulseStart = System.currentTimeMillis();
            } else if (!feedOn && pulseSec >= FEED_OFF_TIME) {
                feedOn = true;
                pulseStart = System.currentTimeMillis();
            }

            if (feedOn && atSpeed) {
                intake.setPower(0.4);
                feeder.setPower(0.4);
            } else {
                intake.setPower(0.0);
                feeder.setPower(0.0);
            }

            idle();
        }

        // Extra clear
        long clearStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - clearStart < CLEAR_AFTER_MS) {
            intake.setPower(0.4);
            feeder.setPower(0.4);
            idle();
        }

        stopIntake();
        launcher.setVelocity(0);
        sleep(150);
    }

    private double computeBaseScalar(double d) {
        if (d < 0 || d > MAX_DIST_FORCE) return lastBaseScalar;

        if (d < CLOSE_DIST) {
            return CLOSE_PWR;
        } else if (d < MID_DIST) {
            return interpolate(d, CLOSE_DIST, CLOSE_PWR, MID_DIST, MID_PWR);
        } else if (d < FAR_DIST) {
            return interpolate(d, MID_DIST, MID_PWR, FAR_DIST, FAR_PWR);
        } else {
            return FAR_PWR;
        }
    }

    private double interpolate(double x, double x1, double y1, double x2, double y2) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    private void safeStopAll() {
        try { launcher.setVelocity(0); } catch (Exception ignored) {}
        try { launcher.setPower(0); } catch (Exception ignored) {}
        stopIntake();
    }

    // =========================
    // Stable distance cache
    // =========================
    static class TagDistanceCache {
        private final AprilTagProcessor proc;
        private final long maxAgeMs;
        private final double alpha;

        private double filtered = -1;
        private long lastGoodMs = 0;

        TagDistanceCache(AprilTagProcessor proc, long maxAgeMs, double alpha) {
            this.proc = proc;
            this.maxAgeMs = maxAgeMs;
            this.alpha = alpha;
        }

        void update() {
            List<AprilTagDetection> detections = proc.getDetections();
            double bestRange = 1e9;

            for (AprilTagDetection d : detections) {
                if (d.ftcPose == null) continue;
                if (d.ftcPose.range < bestRange) bestRange = d.ftcPose.range;
            }

            if (bestRange < 1e9) {
                double raw = bestRange * 100.0; // your scaling
                long now = System.currentTimeMillis();

                if (filtered < 0) filtered = raw;
                else filtered = filtered + alpha * (raw - filtered);

                lastGoodMs = now;
            }
        }

        double getStableDistance() {
            long now = System.currentTimeMillis();
            if (filtered < 0) return -1;
            if (now - lastGoodMs > maxAgeMs) return -1;
            return filtered;
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Disabled
@Autonomous(name = "CycloneAutoRed_RR", group = "Robot")
public class RedAutoRR extends LinearOpMode {

    // =======================
    // START POSE (EDIT THIS)
    // =======================
    // X forward, Y left (RR quickstart convention), inches, heading radians.
    private static final Pose2d START_POSE = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));

    // =======================
    // SIMPLE DISTANCES (EDIT)
    // =======================
    private static final double BACK_UP_IN = 47;      // move backward from start
    private static final double INTAKE_FWD_IN = 35;   // forward while intaking
    private static final double TURN_LEFT_DEG = -165; // rotate left
    private static final double TURN_RIGHT_DEG = 165; // rotate right back

    // =======================
    // Shooter (TeleOp-style velocity)
    // =======================
    // Motor constants (matches your TeleOp file)
    private static final double TICKS_PER_REV = 28.0; // go 5202/5203
    private static final double MAX_RPM = 5200;
    private static final double RPM_TOLERANCE = 200;

    // Distance thresholds (same idea as your existing Auto/TeleOp)
    private static final double CLOSE_DIST = 3000;
    private static final double MID_DIST   = 3900;
    private static final double FAR_DIST   = 5500;

    // Distance → RPM scalars (copied from your TeleOp)
    // NOTE: These are scalars (0..1) that get multiplied by MAX_RPM.
    // Tune these numbers on-field if needed.
    private static final double CLOSE_PWR = 0.33;
    private static final double MID_PWR   = 0.43;
    private static final double FAR_PWR   = 0.53;

    private static final double MAX_DIST_FORCE = 8000;

    // Spin up + shooting window
    private static final long SPINUP_TIMEOUT_MS = 1200; // max wait for RPM
    private static final long SHOOT_WINDOW_MS   = 4000; // main shoot time
    private static final long CLEAR_AFTER_MS    = 250;  // extra clear so no balls remain

    // Feed pulsing (keeps your Auto style but gated by RPM)
    private static final double FEED_ON_TIME  = 1; // seconds
    private static final double FEED_OFF_TIME = 0;  // seconds

    // If tag goes stale, we reuse last known scalar
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

        // --------- Motors ----------
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        feeder = hardwareMap.get(DcMotor.class, "intakeMotor2");

        // --------- Vision (AprilTag) ----------
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logicam"))
                .addProcessor(aprilTag)
                .build();

        BlueAutoRR.TagDistanceCache tagCache = new BlueAutoRR.TagDistanceCache(aprilTag, 300, 0.25);

        // --------- Road Runner drive ----------
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);


        waitForStart();
        if (isStopRequested()) {
            safeStopAll();
            if (portal != null) portal.close();
            return;
        }

        Pose2d pose = START_POSE;

        // 1) BACK UP
        pose = doLineMoveX(drive, pose, -BACK_UP_IN);

        // 2) SHOOT #1 (camera only for distance -> RPM)
        shootUsingVisionDistance(tagCache);

        // 3) TURN LEFT
        pose = doTurn(drive, pose, Math.toRadians(TURN_LEFT_DEG));

        // 4) DRIVE FORWARD WHILE INTAKING
        startIntake();               // TeleOp-style intake mode
        pose = doLineMoveX(drive, pose, +INTAKE_FWD_IN);
        stopIntake();

        // 5) DRIVE BACK (undo intake move)
        pose = doLineMoveX(drive, pose, -INTAKE_FWD_IN);

        // 6) TURN RIGHT (back to original)
        pose = doTurn(drive, pose, Math.toRadians(TURN_RIGHT_DEG));

        // 7) SHOOT #2
        shootUsingVisionDistance(tagCache);

        pose = doTurn(drive, pose, Math.toRadians(TURN_LEFT_DEG+30));




        pose = doLineMoveX(drive, pose, INTAKE_FWD_IN);

        pose = doLineMoveX(drive, pose, -INTAKE_FWD_IN);

        pose = doTurn(drive, pose, Math.toRadians(-TURN_LEFT_DEG-30));


        // Stop everything
        safeStopAll();
        if (portal != null) portal.close();

        telemetry.addLine("AUTO DONE");
        telemetry.update();
        sleep(250);
    }

    // =========================
    // RR helpers (safe + simple)
    // =========================

    private Pose2d doLineMoveX(MecanumDrive drive, Pose2d startPose, double dxInches) {
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToX(startPose.position.x + dxInches)
                        .build()
        );
        return drive.localizer.getPose();
    }

    private Pose2d doLineMoveY(MecanumDrive drive, Pose2d startPose, double dyInches) {
        // Strafe in field Y: +Y is LEFT, -Y is RIGHT
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToY(startPose.position.y + dyInches)
                        .build()
        );
        return drive.localizer.getPose();
    }

    private Pose2d doTurn(MecanumDrive drive, Pose2d startPose, double dThetaRad) {
        if (Math.abs(dThetaRad) < 1e-3) return startPose;

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .turn(dThetaRad)
                        .build()
        );
        return drive.localizer.getPose();
    }

    // =========================
    // Intake helpers (TeleOp-style)
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
    // SHOOT ROUTINE (TeleOp shooter logic -> Auto)
    // =========================

    private void shootUsingVisionDistance(BlueAutoRR.TagDistanceCache tagCache) {
        // Stabilize distance reads
        for (int i = 0; i < 10; i++) {
            tagCache.update();
            sleep(20);
        }

        double d = tagCache.getStableDistance(); // -1 if stale/unavailable

        // TeleOp behavior: if distance invalid, reuse last scalar
        double baseScalar = computeBaseScalar(d);
        if (d >= 0) lastBaseScalar = baseScalar;

        double targetRPM = baseScalar * MAX_RPM;

        telemetry.addData("Step", "SHOOT (Velocity)");
        telemetry.addData("StableDistance", d);
        telemetry.addData("BaseScalar", baseScalar);
        telemetry.addData("TargetRPM", targetRPM);
        telemetry.update();

        // Spin shooter to target velocity
        launcher.setVelocity(targetRPM * TICKS_PER_REV / 60.0);

        // Wait until at speed (or timeout)
        waitForShooterAtSpeed(targetRPM, SPINUP_TIMEOUT_MS);

        // Feed for a time window, but ONLY when at speed
        pulseFeedForTimeAtSpeed(SHOOT_WINDOW_MS, targetRPM);

        // Extra clear to avoid balls left in robot
        pulseFeedForTimeAtSpeed(CLEAR_AFTER_MS, targetRPM);

        // Shutdown
        stopIntake();
        launcher.setVelocity(0);

        sleep(200);
    }

    private double computeBaseScalar(double d) {
        // If out of range / stale -> use last known
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

    private void waitForShooterAtSpeed(double targetRPM, long timeoutMs) {
        ElapsedTime t = new ElapsedTime();
        t.reset();

        while (opModeIsActive() && t.milliseconds() < timeoutMs) {
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;
            boolean atSpeed = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;

            telemetry.addData("SpinUpRPM", currentRPM);
            telemetry.addData("AtSpeed", atSpeed);
            telemetry.update();

            if (atSpeed) return;
            idle();
        }
    }

    private void pulseFeedForTimeAtSpeed(long totalMs, double targetRPM) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime pulseTimer = new ElapsedTime();

        boolean feedOn = false;
        timer.reset();
        pulseTimer.reset();

        while (opModeIsActive() && timer.milliseconds() < totalMs) {

            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;
            boolean atSpeed = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;

            // Pulse toggle
            if (feedOn && pulseTimer.seconds() >= FEED_ON_TIME) {
                feedOn = false;
                pulseTimer.reset();
            } else if (!feedOn && pulseTimer.seconds() >= FEED_OFF_TIME) {
                feedOn = true;
                pulseTimer.reset();
            }

            // Feed ONLY if pulse is on AND shooter is at speed
            if (feedOn && atSpeed) {
                intake.setPower(0.4);
                feeder.setPower(0.4);
            } else {
                intake.setPower(0.0);
                feeder.setPower(0.0);
            }

            telemetry.addData("FeedingPulse", feedOn);
            telemetry.addData("AtSpeed", atSpeed);
            telemetry.addData("RPM", currentRPM);
            telemetry.addData("FeedTime(ms)", timer.milliseconds());
            telemetry.update();

            idle();
        }

        stopIntake();
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

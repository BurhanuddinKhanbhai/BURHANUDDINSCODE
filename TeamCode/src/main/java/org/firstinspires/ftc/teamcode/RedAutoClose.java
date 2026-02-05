package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "RedAutoClose", group = "RoadRunner")
public class RedAutoClose extends LinearOpMode {

    // =========================
    // Shooter tuning (FROM BLUE - working)
    // =========================
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 5200;

    private static final double CLOSE_DIST = 3000;
    private static final double MID_DIST   = 3900;
    private static final double FAR_DIST   = 5500;

    private static final double CLOSE_PWR = 0.35;
    private static final double MID_PWR   = 0.45;
    private static final double FAR_PWR   = 0.54;

    private static final double MAX_DIST_FORCE = 8000;

    // =========================
    // Velocity-based "stable latch" settings (FROM BLUE - working)
    // =========================
    private static final long READY_TIMEOUT_MS    = 2500;
    private static final long READY_STABLE_MS     = 120;
    private static final double READY_CUSHION_RPM = 200;

    private static final long SHOOT_WINDOW_MS = 1800;
    private static final long CLEAR_AFTER_MS  = 200;

    private static final double FEED_PWR = 1.0;

    private static final boolean KEEP_FLYWHEEL_SPINNING = true;
    private static final double IDLE_SPIN_SCALAR = 0.20;

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

        TagDistanceCache tagCache = new TagDistanceCache(aprilTag, 300, 0.25);

        // 1) Start Pose
        Pose2d startPose = new Pose2d(0, 0, 0);

        // 2) Drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // ==========================================================
        // ============ MOVEMENT (MIRRORED FROM BLUE TO RED) =========
        // Mirror rule used:
        //   Red = flip Y and flip heading/turn sign
        // ==========================================================

        Action begining = drive.actionBuilder(startPose)
                .build();

        // BLUE: start heading +47, strafeTo(-44, -35)
        // RED:  start heading -47, strafeTo(-44, +35)
        Action movementFirstShot = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(-47)))
                .strafeTo(new Vector2d(-44, 35))
                .build();

        // BLUE: from (-44,-40, +47) turn(+47)
        // RED:  from (-44,+40, -47) turn(-47)
        Action movementTurnFirstRow = drive.actionBuilder(new Pose2d(-44, 40, Math.toRadians(-47)))
                .turn(Math.toRadians(-47))
                .build();

        // BLUE: from (-44,-40, +90) strafeTo(-55, +10)
        // RED:  from (-44,+40, -90) strafeTo(-55, -10)
        Action IntakeFirstRow = drive.actionBuilder(new Pose2d(-44, 40, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-55, -10))
                .build();

        // BLUE: from (-55,+10, +47) strafeTo(-44,-40)
        // RED:  from (-55,-10, -47) strafeTo(-44,+40)
        Action MoveBackToShoot = drive.actionBuilder(new Pose2d(-55, -10, Math.toRadians(-47)))
                .strafeTo(new Vector2d(-44, 40))
                .build();

        // BLUE: from (-44,-40, +90) strafeTo(-83,-40)
        // RED:  from (-44,+40, -90) strafeTo(-83,+40)
        Action MoveToSecondRow = drive.actionBuilder(new Pose2d(-44, 40, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-83, 40))
                .build();

        // BLUE: from (-83,-40, +90) strafeTo(-83,+12)
        // RED:  from (-83,+40, -90) strafeTo(-83,-12)
        Action IntakeSecondRow = drive.actionBuilder(new Pose2d(-83, 40, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-83, -12))
                .build();

        // BLUE: from (-83,+12, +45) strafeTo(-45,-45)
        // RED:  from (-83,-12, -45) strafeTo(-45,+45)
        Action ShootFinal = drive.actionBuilder(new Pose2d(-83, -12, Math.toRadians(-45)))
                .strafeTo(new Vector2d(-45, 45))
                .build();

        // ==========================================================
        // ===================== FULL AUTO ==========================
        // ==========================================================

        Action fullAuto = new SequentialAction(
                begining,

                movementFirstShot,
                shootUsingVisionAction(tagCache),

                movementTurnFirstRow,
                startIntakeAction(1.0),
                IntakeFirstRow,
                startIntakeAction(0),

                MoveBackToShoot,
                shootUsingVisionAction(tagCache),

                MoveToSecondRow,
                startIntakeAction(1.0),
                IntakeSecondRow,
                startIntakeAction(0),

                ShootFinal,
                shootUsingVisionAction(tagCache),

                safeStopAllAction()
        );

        telemetry.addLine("Ready: RedAutoClose (mirrored from BlueAutoClose)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            Actions.runBlocking(fullAuto);
        } finally {
            safeStopAll();
            if (portal != null) portal.close();
        }
    }

    // ==========================================================
    // ===================== ACTION HELPERS ======================
    // ==========================================================

    private Action startIntakeAction(double power) {
        return p -> {
            intake.setPower(power);
            // IMPORTANT: If you need feeder during intake, change this to feeder.setPower(power);
            feeder.setPower(0.0);
            return false;
        };
    }

    private Action shootUsingVisionAction(TagDistanceCache tagCache) {
        return new Action() {
            boolean done = false;

            @Override
            public boolean run(TelemetryPacket p) {
                if (done) return false;
                shootUsingVisionDistance(tagCache);
                done = true;
                return false;
            }
        };
    }

    private Action safeStopAllAction() {
        return p -> {
            safeStopAll();
            return false;
        };
    }

    // ==========================================================
    // ===================== SHOOTING LOGIC ======================
    // ==========================================================

    private void stopIntake() {
        intake.setPower(0.0);
        feeder.setPower(0.0);
    }

    private void shootUsingVisionDistance(TagDistanceCache tagCache) {

        for (int i = 0; i < 10; i++) {
            tagCache.update();
            sleep(20);
        }

        double d = tagCache.getStableDistance();

        double baseScalar = computeBaseScalar(d);
        if (d >= 0) lastBaseScalar = baseScalar;

        double targetRPM = baseScalar * MAX_RPM;

        telemetry.addData("Step", "SHOOT (velocity stable latch)");
        telemetry.addData("StableDistance", d);
        telemetry.addData("BaseScalar", baseScalar);
        telemetry.addData("TargetRPM", targetRPM);
        telemetry.update();

        launcher.setVelocity(targetRPM * TICKS_PER_REV / 60.0);

        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        ElapsedTime stable = new ElapsedTime();
        boolean stableTiming = false;

        boolean latchedReady = false;

        while (opModeIsActive() && timeout.milliseconds() < READY_TIMEOUT_MS) {
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;
            boolean readyNow = currentRPM >= (targetRPM - READY_CUSHION_RPM);

            if (readyNow) {
                if (!stableTiming) {
                    stableTiming = true;
                    stable.reset();
                }
                if (stable.milliseconds() >= READY_STABLE_MS) {
                    latchedReady = true;
                    break;
                }
            } else {
                stableTiming = false;
            }

            telemetry.addData("RPM", currentRPM);
            telemetry.addData("ReadyNow", readyNow);
            telemetry.addData("Stable(ms)", stableTiming ? stable.milliseconds() : 0);
            telemetry.update();
            idle();
        }

        if (!latchedReady) {
            telemetry.addLine("READY TIMEOUT -> best effort feed");
            telemetry.update();
        }

        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        while (opModeIsActive() && shootTimer.milliseconds() < SHOOT_WINDOW_MS) {
            intake.setPower(FEED_PWR);
            feeder.setPower(FEED_PWR);
            idle();
        }

        ElapsedTime clearTimer = new ElapsedTime();
        clearTimer.reset();
        while (opModeIsActive() && clearTimer.milliseconds() < CLEAR_AFTER_MS) {
            intake.setPower(FEED_PWR);
            feeder.setPower(FEED_PWR);
            idle();
        }

        stopIntake();

        if (KEEP_FLYWHEEL_SPINNING) {
            double idleRPM = IDLE_SPIN_SCALAR * MAX_RPM;
            launcher.setVelocity(idleRPM * TICKS_PER_REV / 60.0);
        } else {
            launcher.setVelocity(0);
        }

        sleep(100);
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
                double raw = bestRange * 100.0;
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

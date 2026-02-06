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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "RedAutoFar", group = "RoadRunner")
public class RedAutoFar extends LinearOpMode {

    // =========================
    // Shooter constants (match your TeleOp style)
    // =========================
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 5200;

    // "Far shooting" = LB fixed power in TeleOp (.80)
    private static final double FAR_FIXED_POWER = 0.80;
    private static final double FAR_FIXED_TARGET_RPM = FAR_FIXED_POWER * MAX_RPM;

    // Wait until it hits 4000 RPM before feeding
    private static final double READY_RPM = 3200;

    // Timeout so it doesn't hang forever
    private static final long SPINUP_TIMEOUT_MS = 4000;

    // Feed window (tune for how many balls you want)
    private static final long SHOOT_WINDOW_MS = 1800;
    private static final long CLEAR_AFTER_MS  = 200;
    private static final double FEED_PWR = 1.0;

    // Keep flywheel spinning a bit between shots
    private static final boolean KEEP_FLYWHEEL_SPINNING = true;
    private static final double IDLE_SPIN_SCALAR = 0.20;

    // =========================
    // Hood (from your TeleOp)
    // =========================
    private static final double HOOD_SHOOT_POS = 0.78;
    private static final double HOOD_DEFAULT_POS = 0.65;

    // =======================
    // Hardware
    // =======================
    private DcMotorEx launcher;
    private DcMotor intake, feeder;
    private Servo hoodL, hoodR;

    // =======================
    // Vision (kept in file, not used for .80 mode)
    // =======================
    private VisionPortal portal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // --- Motors ---
        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        feeder = hardwareMap.get(DcMotor.class, "intakeMotor2");

        // --- Hood servos (same names as TeleOp: "hood" and "gate") ---
        hoodL = hardwareMap.get(Servo.class, "hood");
        hoodR = hardwareMap.get(Servo.class, "gate");
        hoodL.setPosition(HOOD_DEFAULT_POS);
        hoodR.setPosition(HOOD_DEFAULT_POS);

        // --- AprilTag vision (left in because your auto had it) ---
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logicam"))
                .addProcessor(aprilTag)
                .build();

        // --- RR drive ---
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // ==========================================================
        // ===================== MOVEMENT (MIRRORED FOR RED) =========
        // Mirror rules used:
        //  - Flip Y coordinates (y -> -y)
        //  - Flip headings/angles (theta -> -theta)
        // ==========================================================

        Action begining = drive.actionBuilder(startPose)
                .build();

        // Blue: turnTo(+12), strafeTo(0,0)
        // Red : turnTo(-12), strafeTo(0,0)
        Action movementFirstShot = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .turnTo(Math.toRadians(-12))
                .strafeTo(new Vector2d(0, 0))
                .build();

        // Blue: startPose (0,0, 90), strafeTo(44,0) then (44, 70)
        // Red : startPose (0,0,-90), strafeTo(44,0) then (44,-70)
        Action IntakeFirstRow = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(-90)))
                .lineToX(25)
                .lineToY(-25)
                .build();

        // Blue: from (44,70, 12) -> (10,10)
        // Red : from (44,-70,-12) -> (10,-10)
        Action MoveBackToShoot = drive.actionBuilder(new Pose2d(25, -25, Math.toRadians(-90)))
                .lineToX(0)
                .lineToY(0)
                .turnTo(Math.toRadians(-12))

                .build();

        // Blue: from (10,10, 165) -> (30,60)
        // Red : from (10,-10,-165) -> (30,-60)
        Action MoveToSecondRow = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(-165)))
                .strafeTo(new Vector2d(30, -60))
                .build();

        // Blue: from (30,60,165) -> (-20,0)
        // Red : from (30,-60,-165) -> (-20,0)
        Action IntakeSecondRow = drive.actionBuilder(new Pose2d(30, -60, Math.toRadians(-165)))
                .strafeTo(new Vector2d(-20, 0))
                .build();

        // Blue: from (-20,0,12) -> (10,10)
        // Red : from (-20,0,-12) -> (10,-10)
        Action ShootFinal = drive.actionBuilder(new Pose2d(-20, 0, Math.toRadians(-12)))
                .strafeTo(new Vector2d(10, -10))
                .build();

        // ==========================================================
        // ===================== FULL AUTO ==========================
        // ==========================================================

        Action fullAuto = new SequentialAction(
                begining,

                movementFirstShot,
                farShootAction(),

                startIntakeAction(1.0),
                IntakeFirstRow,

                MoveBackToShoot,
                startIntakeAction(0.0),
                farShootAction(),

                MoveToSecondRow,
                startIntakeAction(1.0),
                IntakeSecondRow,
                startIntakeAction(0.0),

                ShootFinal,
                farShootAction(),

                safeStopAllAction()
        );

        telemetry.addLine("Ready: RedAutoFar (.80 fixed far shooting, waits >= 4000 RPM)");
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
            // transfer stays OFF while intaking (your request)
            feeder.setPower(0.0);
            return false;
        };
    }

    private Action farShootAction() {
        return new Action() {
            boolean done = false;

            @Override
            public boolean run(TelemetryPacket p) {
                if (done) return false;
                farShootFixedPointEight();
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
    // ===================== SHOOTING (LB STYLE) =================
    // ==========================================================

    private void stopIntake() {
        intake.setPower(0.0);
        feeder.setPower(0.0);
    }

    private void farShootFixedPointEight() {

        // Hood to shoot pos
        hoodL.setPosition(HOOD_SHOOT_POS);
        hoodR.setPosition(HOOD_SHOOT_POS);

        // Command fixed shooter velocity
        launcher.setVelocity(FAR_FIXED_TARGET_RPM * TICKS_PER_REV / 60.0);

        // Wait for >= READY_RPM
        ElapsedTime spinTimer = new ElapsedTime();
        spinTimer.reset();

        while (opModeIsActive() && spinTimer.milliseconds() < SPINUP_TIMEOUT_MS) {
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;

            telemetry.addData("ShootMode", "FAR_FIXED_.80");
            telemetry.addData("TargetRPM", FAR_FIXED_TARGET_RPM);
            telemetry.addData("CurrentRPM", currentRPM);
            telemetry.addData("ReadyRPM", READY_RPM);
            telemetry.update();

            if (currentRPM >= READY_RPM) break;
            idle();
        }

        // Feed window (NOW transfer ON because you're shooting)
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        while (opModeIsActive() && shootTimer.milliseconds() < SHOOT_WINDOW_MS) {
            intake.setPower(FEED_PWR);
            feeder.setPower(FEED_PWR);
            idle();
        }

        // Clear a bit
        ElapsedTime clearTimer = new ElapsedTime();
        clearTimer.reset();

        while (opModeIsActive() && clearTimer.milliseconds() < CLEAR_AFTER_MS) {
            intake.setPower(FEED_PWR);
            feeder.setPower(FEED_PWR);
            idle();
        }

        stopIntake();

        // Keep flywheel spinning between shots
        if (KEEP_FLYWHEEL_SPINNING) {
            double idleRPM = IDLE_SPIN_SCALAR * MAX_RPM;
            launcher.setVelocity(idleRPM * TICKS_PER_REV / 60.0);
        } else {
            launcher.setVelocity(0);
        }

        // Hood back to default
        hoodL.setPosition(HOOD_DEFAULT_POS);
        hoodR.setPosition(HOOD_DEFAULT_POS);

        sleep(100);
    }

    private void safeStopAll() {
        try { launcher.setVelocity(0); } catch (Exception ignored) {}
        try { launcher.setPower(0); } catch (Exception ignored) {}

        stopIntake();

        try {
            hoodL.setPosition(HOOD_DEFAULT_POS);
            hoodR.setPosition(HOOD_DEFAULT_POS);
        } catch (Exception ignored) {}
    }

    // =========================
    // Optional distance cache (kept, not used)
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

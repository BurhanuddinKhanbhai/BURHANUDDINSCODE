package org.firstinspires.ftc.teamcode;
//This is THE teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CycloneTeleop", group="Robot")
public class TESTSHOOTING extends LinearOpMode {

    // =====================
    // Drivetrain
    // =====================
    DcMotor fl, fr, bl, br;

    // =====================
    // Shooter
    // =====================
    DcMotorEx launcher;

    // =====================
    // Intake
    // =====================
    DcMotor intake, feeder;

    // =====================
    // Hood
    // =====================
    Servo hoodL, hoodR;
    double hoodPos = 0.65; // manual/normal hood position (dpad)
    double hoodPosBeforeHold = 0.65;

    // Tune these
    static final double HOOD_SHOOT_POS = 0.78;   // where hood goes while holding X or LB
    static final double HOOD_STEP      = 0.01;

    // =====================
    // Vision
    // =====================
    VisionPortal portal;
    AprilTagProcessor aprilTag;
    TagDistanceCache tagCache;
    double distance = -1;

    // =====================
    // Launcher tuning
    // =====================
    static final double TICKS_PER_REV = 28.0;  // your current value
    static final double MAX_RPM = 5200;

    // Vision-shot tolerance: we treat "too fast" as OK by using >= (target - tol)
    static final double RPM_TOLERANCE = 150;

    // Left bumper mode: fixed power + fixed RPM threshold
    static final double LB_POWER = 0.80;           // requested .8
    static final double LB_READY_RPM = 3000;       // requested 4000 rpm

    // =====================
    // Distance thresholds (your current ones)
    // =====================
    double CLOSE_DIST = 3000;
    double MID_DIST   = 3000;
    double FAR_DIST   = 3250;

    // Distance -> RPM scalars (your current ones)
    double CLOSE_PWR = 0.35;
    double MID_PWR   = 0.45;
    double FAR_PWR   = 0.54;

    double lastBasePower = 0.0;

    // =====================
    // Feed
    // =====================
    static final double FEED_POWER = 1.0; // feed full while holding
    // (we're feeding continuously while held once ready, so no pulsing needed)

    // =====================
    // One-check latch state
    // =====================
    enum ShootMode { NONE, X_VISION, LB_FIXED }
    ShootMode shootMode = ShootMode.NONE;

    boolean xPrev = false;
    boolean lbPrev = false;

    boolean readyLatched = false;     // becomes true ONCE per hold when RPM condition is met
    double latchedTargetRPM = 0.0;    // for X vision mode
    double latchedDistance = -1;

    @Override
    public void runOpMode() {

        // Hardware
        fl = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "frontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "backLeftDrive");
        br = hardwareMap.get(DcMotor.class, "backRightDrive");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        feeder = hardwareMap.get(DcMotor.class, "intakeMotor2");

        hoodL = hardwareMap.get(Servo.class, "hood");
        hoodR = hardwareMap.get(Servo.class, "gate");

        // Camera
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logicam"))
                .addProcessor(aprilTag)
                .build();

        tagCache = new TagDistanceCache(aprilTag, 300);

        telemetry.addLine("Cyclone TeleOp Velocity READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =====================
            // Drive
            // =====================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            double flP = y + x + r;
            double frP = y - x - r;
            double blP = y - x + r;
            double brP = y + x - r;

            double max = Math.max(
                    Math.max(Math.abs(flP), Math.abs(frP)),
                    Math.max(Math.abs(blP), Math.abs(brP))
            );
            if (max > 1.0) {
                flP /= max; frP /= max; blP /= max; brP /= max;
            }

            fl.setPower(flP);
            fr.setPower(frP);
            bl.setPower(blP);
            br.setPower(brP);

            // =====================
            // Manual hood adjust (dpad)
            // =====================
            if (gamepad2.dpad_up) hoodPos += HOOD_STEP;
            if (gamepad2.dpad_down) hoodPos -= HOOD_STEP;
            hoodPos = Math.max(0, Math.min(1, hoodPos));

            // =====================
            // Distance update (always for telemetry)
            // =====================
            tagCache.update();
            distance = tagCache.getStableDistance();

            // Compute live base power (for right_trigger mode + for latching when X is pressed)
            double liveBasePower = computeBasePower(distance);
            if (distance >= 0) lastBasePower = liveBasePower;

            // =====================
            // Edge detection
            // =====================
            boolean xNow = gamepad2.x;
            boolean lbNow = gamepad2.left_bumper;

            boolean xPressed = xNow && !xPrev;
            boolean xReleased = !xNow && xPrev;

            boolean lbPressed = lbNow && !lbPrev;
            boolean lbReleased = !lbNow && lbPrev;

            // =====================
            // Start / Reset latch logic
            // Priority: LB overrides X if both pressed
            // =====================
            if (lbPressed) {
                shootMode = ShootMode.LB_FIXED;
                readyLatched = false;
                latchedTargetRPM = LB_POWER * MAX_RPM;

                hoodPosBeforeHold = hoodPos; // remember manual pos
            }

            if (xPressed && !lbNow) {
                shootMode = ShootMode.X_VISION;
                readyLatched = false;

                // Latch distance ONCE
                latchedDistance = distance;

                // Latch target RPM ONCE (no trigger scaling)
                double base = computeBasePower(latchedDistance);
                if (latchedDistance >= 0) lastBasePower = base;
                latchedTargetRPM = base * MAX_RPM;

                hoodPosBeforeHold = hoodPos; // remember manual pos
            }

            // Release resets (only if releasing the active mode)
            if (lbReleased && shootMode == ShootMode.LB_FIXED) {
                resetShooterState();
            }
            if (xReleased && shootMode == ShootMode.X_VISION) {
                resetShooterState();
            }

            // =====================
            // Hood behavior while holding buttons
            // =====================
            boolean holdingShootButton = (shootMode == ShootMode.X_VISION && xNow) ||
                    (shootMode == ShootMode.LB_FIXED && lbNow);

            if (holdingShootButton) {
                hoodL.setPosition(HOOD_SHOOT_POS);
                hoodR.setPosition(HOOD_SHOOT_POS);
            } else {
                hoodL.setPosition(hoodPos);
                hoodR.setPosition(hoodPos);
            }

            // =====================
            // Shooter target selection
            // =====================
            double targetRPM = 0;

            if (shootMode == ShootMode.LB_FIXED && lbNow) {
                // Fixed .8 speed
                targetRPM = LB_POWER * MAX_RPM;
            }
            else if (shootMode == ShootMode.X_VISION && xNow) {
                // Vision-latched target
                targetRPM = latchedTargetRPM;
            }
            else {
                // Normal manual mode (right trigger)
                if (gamepad2.left_trigger > 0.1) {
                    targetRPM = 0;
                } else {
                    targetRPM = gamepad2.right_trigger * liveBasePower * MAX_RPM;
                }
            }

            launcher.setVelocity(targetRPM * TICKS_PER_REV / 60.0);

            // =====================
            // ONE-CHECK ready latch (only until it becomes ready, then never check again until reset)
            // =====================
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;

            if (!readyLatched) {
                if (shootMode == ShootMode.X_VISION && xNow) {
                    // ready if we're at/above target (treat overshoot as OK)
                    if (currentRPM >= (latchedTargetRPM - RPM_TOLERANCE)) {
                        readyLatched = true;
                    }
                } else if (shootMode == ShootMode.LB_FIXED && lbNow) {
                    // requested: latch ready when we hit 4000 rpm (or above)
                    if (currentRPM >= LB_READY_RPM) {
                        readyLatched = true;
                    }
                }
            }

            // =====================
            // Feeding logic
            // Once readyLatched is true, feed continuously while holding the button
            // =====================
            boolean shouldFeed =
                    (shootMode == ShootMode.X_VISION && xNow && readyLatched) ||
                            (shootMode == ShootMode.LB_FIXED && lbNow && readyLatched);

            // Intake/feeder manual controls (only when not actively feeding a shot)
            if (shouldFeed) {
                intake.setPower(FEED_POWER);
                feeder.setPower(FEED_POWER);
            }
            else if (gamepad2.a) {
                intake.setPower(1.0);
                feeder.setPower(0.0);
            }
            else if (gamepad2.y) {
                intake.setPower(0.75);
                feeder.setPower(0.75);
            }
            else if (gamepad2.b) {
                intake.setPower(-0.2);
                feeder.setPower(-1.0);
            }
            else {
                intake.setPower(0.0);
                feeder.setPower(0.0);
            }

            // =====================
            // Telemetry
            // =====================
            telemetry.addData("Mode", shootMode);
            telemetry.addData("ReadyLatched", readyLatched);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Target RPM", targetRPM);

            telemetry.addData("Distance", distance);
            telemetry.addData("LatchedDistance", latchedDistance);
            telemetry.addData("LatchedTargetRPM", latchedTargetRPM);

            telemetry.addData("Hood (manual)", hoodPos);
            telemetry.update();

            xPrev = xNow;
            lbPrev = lbNow;
        }

        // Shutdown
        resetShooterState();
        if (portal != null) portal.close();
    }

    private void resetShooterState() {
        shootMode = ShootMode.NONE;
        readyLatched = false;
        latchedTargetRPM = 0.0;
        latchedDistance = -1;

        launcher.setVelocity(0);
        intake.setPower(0);
        feeder.setPower(0);

        // restore hood to manual value
        hoodPos = hoodPosBeforeHold;
        hoodL.setPosition(hoodPos);
        hoodR.setPosition(hoodPos);
    }

    private double computeBasePower(double d) {
        // If we don't have a valid distance, reuse last base power
        if (d < 0) return lastBasePower;

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

    double interpolate(double x, double x1, double y1, double x2, double y2) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}

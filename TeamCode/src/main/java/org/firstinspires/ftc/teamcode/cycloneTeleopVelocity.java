package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name="CycloneTeleopVelocity", group="Robot")
public class cycloneTeleopVelocity extends LinearOpMode {

    // Drivetrain
    DcMotor fl, fr, bl, br;

    // Shooter
    DcMotorEx launcher;

    // Intake
    DcMotor intake, feeder;

    // Hood
    Servo hoodL, hoodR;
    double hoodPos = 0.65;

    // Vision
    VisionPortal portal;
    AprilTagProcessor aprilTag;
    TagDistanceCache tagCache;
    double distance = -1;

    // Launcher tuning
    static final double TICKS_PER_REV = 28.0;     // goBILDA 5202/5203
    static final double MAX_RPM = 5200;
    static final double RPM_TOLERANCE = 500;

    // Distance thresholds
    double CLOSE_DIST = 3000;
    double MID_DIST   = 3000;
    double FAR_DIST   = 3250;

    // Distance -> RPM scalars
    double CLOSE_PWR = .35;
    double MID_PWR   = .45;
    double FAR_PWR   = .56;

    double lastBasePower = 0.0;

    // Feed pulsing
    ElapsedTime feedTimer = new ElapsedTime();
    boolean feedOn = false;

    static final double FEED_ON_TIME  = 1;
    static final double FEED_OFF_TIME = 0;

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

        feedTimer.reset();

        while (opModeIsActive()) {

            // Drive
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
                flP /= max;
                frP /= max;
                blP /= max;
                brP /= max;
            }

            fl.setPower(flP);
            fr.setPower(frP);
            bl.setPower(blP);
            br.setPower(brP);

            // Hood
            if (gamepad1.dpad_up) hoodPos += 0.01;
            if (gamepad1.dpad_down) hoodPos -= 0.01;
            hoodPos = Math.max(0, Math.min(1, hoodPos));
            hoodL.setPosition(hoodPos);
            hoodR.setPosition(hoodPos);

            // Distance
            tagCache.update();
            distance = tagCache.getStableDistance();

            // Distance → basePower
            double basePower;
            if (distance < 0) {
                basePower = lastBasePower;
            } else if (distance < CLOSE_DIST) {
                basePower = CLOSE_PWR;
            } else if (distance < MID_DIST) {
                basePower = interpolate(distance, CLOSE_DIST, CLOSE_PWR, MID_DIST, MID_PWR);
            } else if (distance < FAR_DIST) {
                basePower = interpolate(distance, MID_DIST, MID_PWR, FAR_DIST, FAR_PWR);
            } else {
                basePower = FAR_PWR;
            }
            if (distance >= 0) lastBasePower = basePower;

            // Launcher Velocity
            double currentRPM = launcher.getVelocity() * 60.0 / TICKS_PER_REV;
            double targetRPM;

            if (gamepad1.left_trigger > 0.1) {
                targetRPM = 0;
            } else if (gamepad1.left_bumper) {
                targetRPM = FAR_DIST;
            } else {
                targetRPM = gamepad1.right_trigger * basePower * MAX_RPM;
            }

            launcher.setVelocity(targetRPM * TICKS_PER_REV / 60.0);

            boolean atSpeed = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;

            // Intake / Feed
            if (gamepad1.x && atSpeed) {

                if (feedOn && feedTimer.seconds() >= FEED_ON_TIME) {
                    feedOn = false;
                    feedTimer.reset();
                } else if (!feedOn && feedTimer.seconds() >= FEED_OFF_TIME) {
                    feedOn = true;
                    feedTimer.reset();
                }

                if (feedOn) {
                    intake.setPower(1);
                    feeder.setPower(1);
                } else {
                    intake.setPower(0);
                    feeder.setPower(0);
                }
            }
            else if (gamepad1.a) {
                intake.setPower(1.0);
                feeder.setPower(0);
                feedOn = false;
                feedTimer.reset();
            }
            else if (gamepad1.y) {
                intake.setPower(.75);
                feeder.setPower(.75);
                feedOn = false;
                feedTimer.reset();
            }
            else if (gamepad1.b) {
                intake.setPower(-0.2);
                feeder.setPower(-1.0);
                feedOn = false;
                feedTimer.reset();
            }
            else {
                intake.setPower(0);
                feeder.setPower(0);
                feedOn = false;
                feedTimer.reset();
            }

            // ===== Telemetry =====
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("At Speed", atSpeed);
            telemetry.addData("Distance", distance);
            telemetry.addData("Hood", hoodPos);
            telemetry.update();
        }

        launcher.setVelocity(0);
        intake.setPower(0);
        feeder.setPower(0);
        if (portal != null) portal.close();
    }

    double interpolate(double x, double x1, double y1, double x2, double y2) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}

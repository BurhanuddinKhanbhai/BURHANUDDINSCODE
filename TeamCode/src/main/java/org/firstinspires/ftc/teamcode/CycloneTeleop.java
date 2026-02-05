package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import org.firstinspires.ftc.teamcode.TagDistanceCache;


import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="CycloneTeleop", group="Robot")
public class CycloneTeleop extends LinearOpMode {

    // ===== Drivetrain =====
    DcMotor fl, fr, bl, br;

    // ===== Shooter =====
    DcMotorEx launcher;

    // ===== Intakes =====
    DcMotor intake;
    DcMotor feeder;

    // ===== Hood =====
    Servo hoodL, hoodR;
    double hoodPos = 0.33;

    // ===== Camera =====
    VisionPortal portal;
    AprilTagProcessor aprilTag;
    double distance = -1;
    TagDistanceCache tagCache;

    // Optional: hold last good basePower so it doesn’t jump when tag drops
    double lastBasePower = 0.0;


    // ===== TUNING VALUES =====
    double CLOSE_DIST = 3000;
    double MID_DIST   = 3900;
    double FAR_DIST   = 5500;

    double CLOSE_PWR = 0.56;
    double MID_PWR   = 0.58;
    double FAR_PWR   = 0.65;

    @Override
    public void runOpMode() {

        // ===== Hardware Init =====
        fl = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        fr = hardwareMap.get(DcMotor.class, "frontRightDrive");
        bl = hardwareMap.get(DcMotor.class, "backLeftDrive");
        br = hardwareMap.get(DcMotor.class, "backRightDrive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcherMotor");

        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        feeder = hardwareMap.get(DcMotor.class, "intakeMotor2");

        hoodL = hardwareMap.get(Servo.class, "hood");
        hoodR = hardwareMap.get(Servo.class, "gate");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        // ===== Camera =====
        aprilTag = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                        org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                        "logicam"))
                .addProcessor(aprilTag)
                .build();

        tagCache = new TagDistanceCache(aprilTag, 300); // trust last tag for 300ms


        waitForStart();

        while (opModeIsActive()) {



            // ===== Drive =====
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

            // ===== Hood (UNCHANGED) =====
            if (gamepad1.dpad_up) hoodPos += 0.01;
            else if (gamepad1.dpad_down) hoodPos -= 0.01;
            hoodPos = Math.max(0, Math.min(1, hoodPos));
            hoodL.setPosition(hoodPos);
            hoodR.setPosition(hoodPos);

            // ===== Distance from AprilTag =====
            tagCache.update();
            distance = tagCache.getStableDistance();



            // ===== Smooth Shooter Power Calculation =====
            // ===== Smooth Shooter Power Calculation =====
            double basePower;

            if (distance < 0) {
                // tag stale/not seen recently: HOLD last power instead of snapping
                basePower = lastBasePower;
            }
            else if (distance > 8000) {
                basePower = FAR_PWR;
            }
            else if (distance < CLOSE_DIST) {
                basePower = CLOSE_PWR;
            }
            else if (distance < MID_DIST) {
                basePower = interpolate(distance, CLOSE_DIST, CLOSE_PWR, MID_DIST, MID_PWR);
            }
            else if (distance < FAR_DIST) {
                basePower = interpolate(distance, MID_DIST, MID_PWR, FAR_DIST, FAR_PWR);
            }
            else {
                basePower = FAR_PWR;
            }

// update lastBasePower only when tag is valid
            if (distance >= 0) lastBasePower = basePower;




            // ===== Shooter Control =====
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger  = gamepad1.left_trigger;

            double finalPower;

            if (gamepad1.left_bumper) {
                finalPower = 1;          // override: full power
            } else if (leftTrigger > 0.1) {
                finalPower = -0.2;         // brake/reverse
            } else {
                finalPower = rightTrigger * basePower; // normal shooting power
            }

            launcher.setPower(finalPower);


            // ===== Intake Control =====
            // ===== Intake Control =====

// X = AUTO FEED (pulse)



            if (gamepad1.x) {

                if (feedOn && feedTimer.seconds() >= FEED_ON_TIME) {
                    feedOn = false;
                    feedTimer.reset();
                }
                else if (!feedOn && feedTimer.seconds() >= FEED_OFF_TIME) {
                    feedOn = true;
                    feedTimer.reset();
                }

                if (feedOn) {
                    intake.setPower(1.0);
                    feeder.setPower(1.0);
                } else {
                    intake.setPower(0);
                    feeder.setPower(0);
                }
            }

// A = MANUAL FEED
            else if (gamepad1.a) {
                intake.setPower(1.0);
                feedTimer.reset();
                feedOn = false;
            }
            else if (gamepad1.y) {
                feeder.setPower(1);
            }

// B = REVERSE FEEDER
            else if (gamepad1.b) {
                intake.setPower(-.2);
                feeder.setPower(-1.0);
                feedTimer.reset();
                feedOn = false;
            }

// NOTHING PRESSED
            else {
                intake.setPower(0);
                feeder.setPower(0);
                feedTimer.reset();
                feedOn = false;
            }



            // ===== Telemetry =====
            telemetry.addData("Launcher Vel (ticks/s)", launcher.getVelocity());
            telemetry.addData("Distance", distance);
            telemetry.addData("Hood Pos", hoodPos);
            telemetry.addData("TagAge(ms)", tagCache.getAgeMs());
            telemetry.addData("TagDist(last)", tagCache.getLastGoodDistance());
            telemetry.update();


        }
    }

    // ===== Linear Interpolation Function =====
    double interpolate(double x, double x1, double y1, double x2, double y2) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    ElapsedTime feedTimer = new ElapsedTime();
    boolean feedOn = false;

    static final double FEED_ON_TIME = 0.025;   // seconds
    static final double FEED_OFF_TIME = 0.5;  // seconds


    static final double VELOCITY_TOLERANCE = 75; // ticks/sec below target allowed

}

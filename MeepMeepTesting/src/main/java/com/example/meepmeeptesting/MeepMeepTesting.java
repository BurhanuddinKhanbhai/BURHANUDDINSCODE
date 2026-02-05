package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // =========================================================
    // PICK YOUR ORIGIN HERE (in NORMAL FIELD COORDS)
    // This is where your "triangle = (0,0,0)" actually is on the field.
    // Units: inches, same as RR/MeepMeep.
    // Heading: where your triangle frame's +X points.
    // =========================================================
    static final Pose2d ORIGIN_FIELD = new Pose2d(
            60, -11, Math.toRadians(90) // <-- EXAMPLE ONLY: change this
    );

    // ---- Helpers: local (triangle-space) -> field (meepmeep-space) ----
    static Vector2d L(double x, double y) {
        // Rotate local vector by origin heading, then translate by origin position
        double h = ORIGIN_FIELD.heading.toDouble();
        double cos = Math.cos(h);
        double sin = Math.sin(h);

        double xr = x * cos - y * sin;
        double yr = x * sin + y * cos;

        return new Vector2d(ORIGIN_FIELD.position.x + xr, ORIGIN_FIELD.position.y + yr);
    }

    static Pose2d LP(double x, double y, double headingRad) {
        // Local pose -> field pose
        Vector2d p = L(x, y);
        double h = ORIGIN_FIELD.heading.toDouble() + headingRad;
        return new Pose2d(p.x, p.y, h);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        15)
                .build();

        // =========================================================
        // NOW YOU CAN THINK IN LOCAL COORDS:
        // startPoseLocal = (0,0,0) means "on the triangle, facing your local +X"
        // =========================================================
        Pose2d startPose = LP(0, 0, 0);

        bot.runAction(
                bot.getDrive().actionBuilder(startPose)

                        // ===== Example path written in LOCAL coords =====
                        // Replace these with your auto’s movements but in triangle-space.
                        .turn(Math.toRadians(12))
                        .strafeTo(L(0, 0))

                        .turn(Math.toRadians(90))
                        .strafeTo(L(25, 0))
                        .strafeTo(L(25, 50))

                        .turn(Math.toRadians(12))
                        .strafeTo(L(0, 0))

                        .turn(Math.toRadians(165 - 12))
                        .strafeTo(L(30, 60))

                        .strafeTo(L(-20, 0))

                        .turn(Math.toRadians(12 - 165))
                        .strafeTo(L(10, 10))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}


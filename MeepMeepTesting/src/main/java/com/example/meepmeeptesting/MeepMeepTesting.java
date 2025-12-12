package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.Console;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;



public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // NOTE: In actual autonomous OpMode, initialize subsystems here:
        // LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap, telemetry);
        // Lifter lifter = new Lifter(hardwareMap, telemetry);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-49.5, 49.5, Math.toRadians(126))) //starting position
                        .back(32)
                        .turn(Math.toRadians(5))
                        // In actual autonomous, add these commands here:
                        // launcher.leftMotor.setVelocity(750);
                        // launcher.rightMotor.setVelocity(750);
                        // lifter.setPosition(3);  // Position 3 = 0.
                        .setReversed(true)
                        .splineTo(new Vector2d(-12, 28), Math.toRadians(90))
                        .setReversed(false)


                        .waitSeconds(1)
                        //.back(21, new MecanumVelocityConstraint(12, 18), new ProfileAccelerationConstraint(12))
                        // activate intake while driving forward, make sure to activate top belt slightly (0.25s) about half way through to move up top ball

                        //.splineToLinearHeading(new Pose2d(-40.5, 36.5, Math.toRadians(126)), Math.toRadians(135))
                        // shoot again, angle around 0.7 this time
                        //.strafeLeft(12)
                        .waitSeconds(3)
//
//                        .turn(Math.toRadians(90))
//                        .forward(30)
//                        .turn(Math.toRadians(90))
//                        .forward(30)
//                        .turn(Math.toRadians(90))
//                        .forward(30)
//                        .turn(Math.toRadians(90))
                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\robosharks\\Documents\\field-2025-official.png")); }
        catch(Exception e) {
            String boop = e.getMessage();
            System.out.println(boop);
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 20, Math.toRadians(180))) //starting position
                        // shoot
                        .strafeRight(20)
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
package com.example.MeepMeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // TODO: Edit Dimensions + TrackWidth
        RoadRunnerBotEntity LinSanity = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.TANK)
                .setDimensions(16, 16)
                .setConstraints(75, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        LinSanity.runAction(LinSanity.getDrive().actionBuilder(new Pose2d(-12, -62, Math.toRadians(90)))

                // Move To Shooting Position
                .splineTo(new Vector2d(0, -24), Math.toRadians(90))

                // Read Motif AprilTag And Store Motif
                .waitSeconds(2)

                // Aim Towards Depot
                .turnTo(Math.toRadians(120))

                // Lock On To AprilTag (ONLY HEADING CORRECTION)
                .waitSeconds(2)

                // Shoot Motif In Order
                .waitSeconds(5)

                // Go Towards Nearest Artifact Line
                .setReversed(true)
                .splineTo(new Vector2d(-30, -36), Math.toRadians(180))
                .waitSeconds(1)

                // Collect Artifacts
                .setReversed(false)
                .lineToX(-60)
                .waitSeconds(2)

                // Return To Shooting Position
                .splineTo(new Vector2d(0, -24), Math.toRadians(120))

                // Lock On To AprilTag (ONLY HEADING CORRECTION)
                .waitSeconds(2)

                // Shoot Motif In Order
                .waitSeconds(5)

                // Build
                .build());


        Image img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/MeepMeep/DecodeFieldMap.png"));
        } catch (IOException e) {
        }
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(LinSanity)
                .start();
    }
}
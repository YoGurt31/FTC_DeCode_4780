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

        LinSanity.runAction(LinSanity.getDrive().actionBuilder(new Pose2d(-16, -54, Math.toRadians(90)))

                // Read Motif AprilTag And Store Motif

                // Aim Towards Depot
                .turnTo(Math.toRadians(105))

                // Shoot Motif In Order

                // Go Towards Nearest Artifact Line
                .splineTo(new Vector2d(-16, -36), Math.toRadians(0))
                .waitSeconds(1)

                // Collect Artifacts
                .setReversed(true)
                .lineToX(-60)
                .waitSeconds(2)

                // Return To Shooting Position
                .setReversed(false)
                .splineTo(new Vector2d(-24, -36), Math.toRadians(0))
                .splineTo(new Vector2d(-16, -54), Math.toRadians(105))

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
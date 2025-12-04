package Auton;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import RoadRunner.TankDrive;

@Config
@Autonomous(name = "SampleRR", group = "Autonomous")
public class SampleRR extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder Run = drive.actionBuilder(initialPose)
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
                .waitSeconds(5);

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen = Run.build()
                )
        );
    }
}
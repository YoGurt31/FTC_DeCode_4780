package Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import RoadRunner.TankDrive;
import Systems.Robot;

@Autonomous(name = "AdvancedMotif", group = "Auton")
public class AdvancedMotif extends LinearOpMode {

    private final Robot robot = new Robot();

    // FlyWheel Variables
    private static final double targetRPS = 25.0;
    private static final double TicksPerRev = 4000.0; // FlyWheel ELC Encoder Resolution
    private final double artifactHold = 1.0;
    private final double artifactRelease = 0.0;

    // AprilTag / Vision Variables
    // TODO: Tune Values
    private static final double rotateGain = 0.010;
    private static final double maxRotate = 0.25;

    // Motif Variables
    private int motifID = 0;
    private String motifPattern = "UNKNOWN";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Starting Pose
        Pose2d startPose = new Pose2d(-12, -62, Math.toRadians(90));

        // RoadRunner Tank Drive
        TankDrive drive = new TankDrive(hardwareMap, startPose);
        drive.localizer.setPose(startPose);

        telemetry.addLine("Status: Initialized. Ready to Start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Drive to Shooting Position and Read Motif
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(0, -24), Math.toRadians(90))
                        .build()
        );

        // Read Motif from Limelight + Store Pattern
        motifID = readMotifFromTag();
        telemetry.addData("Motif ID", motifID);
        telemetry.addData("Motif Pattern", motifPattern);
        telemetry.update();

        // Align To Depot + Shoot Motif Pattern
        alignToScoringTag();
        shootBasedOnMotif(motifID);

        // 2) Drive to Nearest Artifact Line
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .setReversed(true)
                        .splineTo(new Vector2d(-30, -36), Math.toRadians(180))
                        .build()
        );

        collectArtifacts();

        // Complete Collection
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(-60)
                        .build()
        );

        // 3) Return to Shooting Position
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(0, -24), Math.toRadians(120))
                        .build()
        );

        alignToScoringTag();
        shootBasedOnMotif(motifID);

        stopAll();
    }

    private int readMotifFromTag() {
        LLResult res = robot.vision.limeLight.getLatestResult();
        if (res != null && res.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                int id = fiducials.get(0).getFiducialId();

                switch (id) {
                    case 21:
                        motifPattern = "GPP";
                        break;
                    case 22:
                        motifPattern = "PGP";
                        break;
                    case 23:
                        motifPattern = "PPG";
                        break;
                    default:
                        motifPattern = "UNKNOWN";
                        break;
                }

                motifID = id;
                return id;
            }
        }
        motifPattern = "UNKNOWN";
        motifID = 0;
        return 0; // Unknown Motif
    }

    private void alignToScoringTag() {
        long alignStart = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - alignStart) < 2500) {
            LLResult current = robot.vision.limeLight.getLatestResult();
            if (current == null || !current.isValid()) break;

            double headingError = current.getTx();
            double rotateCmd = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

            robot.driveTrain.tankDrive(0, rotateCmd);

            telemetry.addData("Align", "Turn %.2f", rotateCmd);
            telemetry.addData("Heading Error", headingError);
            telemetry.update();

            idle();
        }

        robot.driveTrain.tankDrive(0, 0);
    }

    private void chargeFlyWheel() {
        robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
        robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

        long spinStart = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - spinStart) < 2000) {
            double measuredRps = robot.scoringMechanisms.flyWheel1.getVelocity() / TicksPerRev;
            telemetry.addData("Flywheel RPS", measuredRps);
            telemetry.update();
            if (measuredRps >= (targetRPS - 0.5)) break;
            idle();
        }
    }

    private void shootBasedOnMotif(int motif) {
        chargeFlyWheel();

        switch (motif) {
            case 1: // GPP
                robot.scoringMechanisms.leftRelease.setPosition(artifactRelease);
                sleep(1000);
                robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
                robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
                sleep(2000);
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);

                break;

            case 2: // PGP
                robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
                sleep(1000);
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
                robot.scoringMechanisms.leftRelease.setPosition(artifactRelease);
                sleep(1000);
                robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
                robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
                sleep(1000);
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
                break;

            case 3: // PPG
                robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
                sleep(2000);
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
                robot.scoringMechanisms.leftRelease.setPosition(artifactRelease);
                sleep(1000);
                robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
                break;

            default:
                // Unknown Motif
                robot.scoringMechanisms.leftRelease.setPosition(artifactRelease);
                robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
                sleep(3000);
                robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
                break;
        }

        robot.scoringMechanisms.flyWheel1.setPower(0);
        robot.scoringMechanisms.flyWheel2.setPower(0);
    }

    private void collectArtifacts() {
        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(1.0);
        sleep(3000);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }

    private void stopAll() {
        robot.driveTrain.tankDrive(0, 0);
        robot.scoringMechanisms.flyWheel1.setPower(0);
        robot.scoringMechanisms.flyWheel2.setPower(0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
        robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
        try {
            robot.vision.limeLight.stop();
        } catch (Exception ignored) {
        }
    }
}
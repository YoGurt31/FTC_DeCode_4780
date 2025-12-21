package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Systems.Robot;
// TODO: Change this import to match your actual TankDrive package
import RoadRunner.TankDrive;

@Autonomous(name = "AutoBlueRR", group = "Auton")
public class BlueRR extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();
    private TankDrive drive;

    // FlyWheel Variables
    private static final double targetRPS = 57.5;
    private static final double TicksPerRev = 28.0;
    private final double artifactHoldRight = 0.5;
    private final double artifactHoldLeft = 0.0;
    private final double artifactReleaseRight = 1.0;
    private final double artifactReleaseLeft = 0.5;

    // AprilTag / Vision Variables
    private static final double rotateGain = 0.0250;
    private static final double maxRotate = 0.75;
    private static final int blueTargetId = 20;

    // Drive Constants
    private static final double rotatePower = 0.10;

    // Non-Blocking Sleep
    private void waitSec(double sec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < sec) {
            idle();
        }
    }

    private void chargeFlywheel() {
        double targetTicksPerSec = targetRPS * TicksPerRev;

        robot.scoringMechanisms.flyWheel1.setVelocity(targetTicksPerSec);
        robot.scoringMechanisms.flyWheel2.setVelocity(targetTicksPerSec);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 2.0) {
            double measuredRps =
                    Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) / TicksPerRev;

            telemetry.addLine("=== Flywheel Charge ===");
            telemetry.addData("Flywheel RPS", measuredRps);
            telemetry.addData("Target RPS", targetRPS);
            telemetry.update();

            if (measuredRps >= (targetRPS - 0.5)) {
                break;
            }
            idle();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        Pose2d initialPose = new Pose2d(-16, -64, Math.toRadians(90));
        drive = new TankDrive(hardwareMap, initialPose);

        // Active If Using LimeLight
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 30);
        robot.vision.limeLight.setPollRateHz(30);
        robot.vision.limeLight.pipelineSwitch(0);

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("PreLoad Order: L - 2 Purple | R - 1 Green");
        telemetry.addLine("Scanning Motif...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Motif Detection
        robot.vision.updateMotif();

        telemetry.addLine("=== Motif Detection ===");
        telemetry.addData("Has Motif", robot.vision.hasMotif());
        telemetry.addData("Motif Tag ID", robot.vision.motifTagId);
        telemetry.addData("Motif Pattern", robot.vision.motifPattern);
        telemetry.update();

        // Motif Detected or BackUp
        String motif = robot.vision.hasMotif() ? robot.vision.motifPattern : "GPP";

        // Blue Pipeline
        robot.vision.setPipeline(robot.vision.BLUE);

        // TODO: Sequence
        Action toShootingPos = drive.actionBuilder(initialPose)
                .lineToY(-54.0)
                .turnTo(Math.toRadians(105))
                .build();
        Actions.runBlocking(toShootingPos);

        aimAtTag(blueTargetId);

        chargeFlywheel();
        shootMotif(motif);

        Action driveToNearestArtifactLine = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(-24, -36), Math.toRadians(105))
                .turnTo(Math.toRadians(0))
                .build();
        Actions.runBlocking(driveToNearestArtifactLine);

        Action collectArtifactsDrive = drive.actionBuilder(drive.localizer.getPose())
                .setReversed(true)
                .lineToX(-60)
                .build();

        Action collectArtifacts = new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean started = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!started) {
                    timer.reset();
                    started = true;
                }

                double time = timer.seconds();

                // TODO: Tune Timing
                if (time < 1.5) {
                    robot.scoringMechanisms.rollerIntake.setPower(1.0);
                    robot.scoringMechanisms.sorterIntake.setPower(1.0);
                } else if (time < 3.0) {
                    robot.scoringMechanisms.rollerIntake.setPower(1.0);
                    robot.scoringMechanisms.sorterIntake.setPower(-1.0);
                } else {
                    robot.scoringMechanisms.rollerIntake.setPower(1.0);
                    robot.scoringMechanisms.sorterIntake.setPower(0.0);
                }

                boolean stillRunning = collectArtifactsDrive.run(packet);

                if (!stillRunning) {
                    robot.scoringMechanisms.rollerIntake.setPower(0.0);
                    robot.scoringMechanisms.sorterIntake.setPower(0.0);
                }

                return stillRunning;
            }
        };

        Actions.runBlocking(collectArtifacts);

        Action returnToShootingPos = drive.actionBuilder(drive.localizer.getPose())
                .setReversed(false)
                .splineTo(new Vector2d(-36, -36), Math.toRadians(0))
                .splineTo(new Vector2d(-16, -54), Math.toRadians(105))
                .build();
        Actions.runBlocking(returnToShootingPos);

        aimAtTag(blueTargetId);

        chargeFlywheel();
        shootMotif(motif);

        Action driveToNextArtifactLine = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(-24, -12.5), Math.toRadians(105))
                .turnTo(Math.toRadians(0))
                .build();
        Actions.runBlocking(driveToNextArtifactLine);

        // Shutdown
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.driveTrain.tankDrive(0.0, 0.0);
        robot.driveTrain.brake();
        try {
            robot.vision.limeLight.stop();
        } catch (Exception ignored) {
        }
    }

    private void aimAtTag(int desiredTagId) {
        long searchStart = System.currentTimeMillis();
        final long searchTimeOut = 2500;  // 2.5 seconds

        while (opModeIsActive()) {
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasCorrectTag = false;
            int tagId = -1;

            if (result != null && result.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    tagId = fiducials.get(0).getFiducialId();
                    hasCorrectTag = (tagId == desiredTagId);
                }
            }

            if (hasCorrectTag || System.currentTimeMillis() - searchStart > searchTimeOut) {
                break;
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rotatePower));
            drive.updatePoseEstimate();

            telemetry.addLine("=== Aim Phase 1 ===");
            telemetry.addData("Seen Tag ID", tagId);
            telemetry.addData("Desired Tag ID", desiredTagId);
            telemetry.update();

            idle();
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
        drive.updatePoseEstimate();

        long alignStart = System.currentTimeMillis();
        final long alignmentTimeOut = 3000;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasCorrectTag = false;
            double headingError = 0.0;

            if (result != null && result.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult fid = fiducials.get(0);
                    int tagId = fid.getFiducialId();
                    if (tagId == desiredTagId) {
                        hasCorrectTag = true;
                        headingError = result.getTx();
                    }
                }
            }

            if (now - alignStart > alignmentTimeOut) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
                drive.updatePoseEstimate();
                telemetry.addLine("AimAtTag: Alignment Timeout");
                telemetry.update();
                break;
            }

            if (hasCorrectTag) {
                double rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

                if (Math.abs(headingError) < 1.0) {
                    rotate = 0.0;
                }

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rotate));
                drive.updatePoseEstimate();

                telemetry.addLine("=== Aim Phase 2 ===");
                telemetry.addData("Heading Error", headingError);
                telemetry.addData("Rotate Cmd", rotate);
                telemetry.update();

                if (Math.abs(headingError) <= 1.0) {
                    break;
                }
            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
                drive.updatePoseEstimate();
            }
            idle();
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
        drive.updatePoseEstimate();
    }

    private void shootMotif(String motif) {
        telemetry.addLine("=== Shooting Motif ===");
        telemetry.addData("Motif", motif);
        telemetry.update();

        switch (motif) {
            case "GPP":
                fireRight();
                fireLeft();
                fireLeft();
                break;
            case "PGP":
                fireLeft();
                fireRight();
                fireLeft();
                break;
            case "PPG":
                fireLeft();
                fireLeft();
                fireRight();
                break;
            default:
                fireLeft();
                fireLeft();
                fireRight();
                break;
        }

        robot.scoringMechanisms.flyWheel1.setPower(0);
        robot.scoringMechanisms.flyWheel2.setPower(0);
    }

    private void fireLeft() {
        robot.scoringMechanisms.leftRelease.setPosition(artifactReleaseLeft);
        waitSec(0.5);
        robot.scoringMechanisms.leftRelease.setPosition(artifactHoldLeft);
        waitSec(0.25);

        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(1.0);
        waitSec(1.0);

        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }

    private void fireRight() {
        robot.scoringMechanisms.rightRelease.setPosition(artifactReleaseRight);
        waitSec(0.5);
        robot.scoringMechanisms.rightRelease.setPosition(artifactHoldRight);
        waitSec(0.25);

        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(-1.0);
        waitSec(1.0);

        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }
}
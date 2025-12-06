package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Systems.Robot;

@Autonomous(name = "AutoBlue", group = "Auton")
public class AutonBlue extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

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
    private static final int redTargetId = 24;

    // Drive Constants
    private static final double drivePower = 0.25;
    private static final double rotatePower = -0.10;

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

        // Active If Using LimeLight
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 30);
        robot.vision.limeLight.setPollRateHz(30);
        robot.vision.limeLight.pipelineSwitch(0);

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("PreLoad Order: L - 2 Purple | R - 1 Green");
        telemetry.addLine("Scanning Motif...");
        telemetry.update();

        waitForStart();
        // Motif Detection
        robot.vision.updateMotif();

        telemetry.addLine("=== Motif Detection ===");
        telemetry.addData("Has Motif", robot.vision.hasMotif());
        telemetry.addData("Motif Tag ID", robot.vision.motifTagId);
        telemetry.addData("Motif Pattern", robot.vision.motifPattern);
        telemetry.update();

        sleep(1000);

        // Motif Detected or BackUp
        String motif = robot.vision.hasMotif() ? robot.vision.motifPattern : "GPP";

        // Red PipeLine
        robot.vision.setPipeline(robot.vision.RED);

        // TODO: Sequence
        robot.driveTrain.tankDrive(drivePower, 0);
        sleep(1000);
        aimAtTag(redTargetId);
        chargeFlywheel();
        shootMotif(motif);
        sleep(1000);
        robot.driveTrain.tankDrive(drivePower, 0);
        sleep(2000);

        // Shutdown
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.driveTrain.tankDrive(0.0, 0.0);
        robot.driveTrain.brake();
        try {
            robot.vision.limeLight.stop();
        } catch (Exception ignored) {}
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

            robot.driveTrain.tankDrive(0.0, rotatePower);

            telemetry.addLine("=== Aim Phase 1 ===");
            telemetry.addData("Seen Tag ID", tagId);
            telemetry.addData("Desired Tag ID", desiredTagId);
            telemetry.update();

            idle();
        }

        robot.driveTrain.tankDrive(0.0, 0.0);

        long alignStart = System.currentTimeMillis();
        final long alignmentTimeOut = 3000;

        long lastSeenCorrectTagTime = System.currentTimeMillis();

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
                        lastSeenCorrectTagTime = now;
                    }
                }
            }

            if (now - alignStart > alignmentTimeOut) {
                robot.driveTrain.tankDrive(0.0, 0.0);
                telemetry.addLine("AimAtTag: Alignment Timeout");
                telemetry.update();
                break;
            }

            if (hasCorrectTag) {
                double rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

                if (Math.abs(headingError) < 1) {
                    rotate = 0.0;
                }

                robot.driveTrain.tankDrive(0.0, rotate);

                telemetry.addLine("=== Aim Phase 2 ===");
                telemetry.addData("Heading Error", headingError);
                telemetry.addData("Rotate Cmd", rotate);
                telemetry.update();

                if (Math.abs(headingError) <= 1.0) {
                    break;
                }
            } else {
                robot.driveTrain.tankDrive(0.0, 0.0);
            }
            idle();
        }

        robot.driveTrain.tankDrive(0.0, 0.0);
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
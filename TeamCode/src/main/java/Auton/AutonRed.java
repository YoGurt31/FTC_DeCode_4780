package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import Systems.Robot;

@Autonomous(name = "AutoRed Basic", group = "Auton")
public class AutonRed extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // FlyWheel Variables
    private static final double targetRPS = 53.5;
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
    private static final double rotatePower = 0.10;

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
        robot.vision.setPipeline(1);

        // TODO: Sequence
        robot.driveTrain.tankDrive(drivePower, 0);
        sleep(1000);
        aimAtTag(redTargetId);
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
        robot.vision.limeLight.stop();
    }

    private void aimAtTag(int desiredTagId) {
        // ------------------------------
        // Phase 1: Rotate until tag seen
        // ------------------------------
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

            // Found desired tag or timed out searching
            if (hasCorrectTag || System.currentTimeMillis() - searchStart > searchTimeOut) {
                break;
            }

            // Rotate left until the correct tag appears in view
            robot.driveTrain.tankDrive(0.0, rotatePower);

            telemetry.addLine("=== Aim Phase 1 ===");
            telemetry.addData("Seen Tag ID", tagId);
            telemetry.addData("Desired Tag ID", desiredTagId);
            telemetry.update();

            idle();
        }

        // Stop rotation before fine alignment
        robot.driveTrain.tankDrive(0.0, 0.0);

        // ------------------------------
        // Phase 2: Fine alignment with tx
        // ------------------------------
        long alignStart = System.currentTimeMillis();
        final long alignmentTimeOut = 3000;   // 3 seconds max
        final long lostTagTimeOut   = 500;    // if we lose tag for >0.5s, give up

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

            // If we've lost the correct tag for too long, stop aiming
            if (!hasCorrectTag && (now - lastSeenCorrectTagTime > lostTagTimeOut)) {
                robot.driveTrain.tankDrive(0.0, 0.0);
                telemetry.addLine("AimAtTag: Lost Tag - Stopping Aim");
                telemetry.update();
                break;
            }

            // Timeout protection
            if (now - alignStart > alignmentTimeOut) {
                robot.driveTrain.tankDrive(0.0, 0.0);
                telemetry.addLine("AimAtTag: Alignment Timeout");
                telemetry.update();
                break;
            }

            if (hasCorrectTag) {
                // Proportional rotate based on heading error
                double rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

                // Tiny deadzone so we don't micro-oscillate
                if (Math.abs(headingError) < 0.5) {
                    rotate = 0.0;
                }

                robot.driveTrain.tankDrive(0.0, rotate);

                telemetry.addLine("=== Aim Phase 2 ===");
                telemetry.addData("Heading Error", headingError);
                telemetry.addData("Rotate Cmd", rotate);
                telemetry.update();

                // Consider aligned when error is small
                if (Math.abs(headingError) <= 1.0) {
                    break;
                }
            } else {
                // No correct tag right now, just hold still while we wait to see it again
                robot.driveTrain.tankDrive(0.0, 0.0);
            }

            idle();
        }

        // Final stop after alignment attempt
        robot.driveTrain.tankDrive(0.0, 0.0);
    }

    private void shootMotif(String motif) {
        telemetry.addLine("=== Shooting Motif ===");
        telemetry.addData("Motif", motif);
        telemetry.update();

        switch (motif) {
            case "GPP":
                fireRight(false);
                fireLeft(false);
                fireLeft(true);
                break;
            case "PGP":
                fireLeft(false);
                fireRight(false);
                fireLeft(true);
                break;
            case "PPG":
                fireLeft(false);
                fireLeft(false);
                fireRight(true);
                break;
            default:
                fireLeft(false);
                fireLeft(false);
                fireRight(true);
                break;
        }
    }

    private void fireLeft(boolean activeIntake) {
        long spinStart = System.currentTimeMillis();
        final long spinTimeout = 2000;

        while (opModeIsActive()) {
            double rps1 = Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) / TicksPerRev;
            double rps2 = Math.abs(robot.scoringMechanisms.flyWheel2.getVelocity()) / TicksPerRev;
            double avg = (rps1 + rps2) / 2.0;

            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            if (avg >= (targetRPS - 1.0) || System.currentTimeMillis() - spinStart > spinTimeout) {
                break;
            }

            idle();
        }

        final long gateOpenDuration = 250;
        final long shotDuration = 1500;

        long startTime = System.currentTimeMillis();
        long gateCloseTime = startTime + gateOpenDuration;
        long shotEndTime = startTime + shotDuration;

        while (opModeIsActive() && System.currentTimeMillis() < shotEndTime) {
            long now = System.currentTimeMillis();

            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            if (activeIntake) {
                robot.scoringMechanisms.rollerIntake.setPower(1.0);
                robot.scoringMechanisms.sorterIntake.setPower(1.0);
            } else {
                robot.scoringMechanisms.rollerIntake.setPower(0.0);
                robot.scoringMechanisms.sorterIntake.setPower(0.0);
            }

            if (now < gateCloseTime) {
                robot.scoringMechanisms.leftRelease.setPosition(artifactReleaseLeft);
            } else {
                robot.scoringMechanisms.leftRelease.setPosition(artifactHoldLeft);
            }
            robot.scoringMechanisms.rightRelease.setPosition(artifactHoldRight);

            idle();
        }

        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.scoringMechanisms.leftRelease.setPosition(artifactHoldLeft);
        robot.scoringMechanisms.rightRelease.setPosition(artifactHoldRight);
    }

    private void fireRight(boolean activeIntake) {
        long spinStart = System.currentTimeMillis();
        final long spinTimeout = 2000;

        while (opModeIsActive()) {
            double rps1 = Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) / TicksPerRev;
            double rps2 = Math.abs(robot.scoringMechanisms.flyWheel2.getVelocity()) / TicksPerRev;
            double avg  = (rps1 + rps2) / 2.0;

            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            if (avg >= (targetRPS - 1.0) || System.currentTimeMillis() - spinStart > spinTimeout) {
                break;
            }

            idle();
        }

        final long gateOpenDuration = 250;
        final long shotDuration     = 1500;

        long startTime     = System.currentTimeMillis();
        long gateCloseTime = startTime + gateOpenDuration;
        long shotEndTime   = startTime + shotDuration;

        while (opModeIsActive() && System.currentTimeMillis() < shotEndTime) {
            long now = System.currentTimeMillis();

            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            if (activeIntake) {
                robot.scoringMechanisms.rollerIntake.setPower(1.0);
                robot.scoringMechanisms.sorterIntake.setPower(-1.0);
            } else {
                robot.scoringMechanisms.rollerIntake.setPower(0.0);
                robot.scoringMechanisms.sorterIntake.setPower(0.0);
            }

            if (now < gateCloseTime) {
                robot.scoringMechanisms.rightRelease.setPosition(artifactReleaseRight);
            } else {
                robot.scoringMechanisms.rightRelease.setPosition(artifactHoldRight);
            }
            robot.scoringMechanisms.leftRelease.setPosition(artifactHoldLeft);

            idle();
        }

        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.scoringMechanisms.leftRelease.setPosition(artifactHoldLeft);
        robot.scoringMechanisms.rightRelease.setPosition(artifactHoldRight);
    }
}
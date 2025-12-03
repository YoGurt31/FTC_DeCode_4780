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
    private static final double TicksPerRev = 28.0; // FlyWheel Encoder Resolution
    private final double artifactHoldRight = 0.5;
    private final double artifactHoldLeft = 0.0;
    private final double artifactReleaseRight = 1.0;
    private final double artifactReleaseLeft = 0.5;

    // AprilTag / Vision Variables
    private static final double rotateGain = 0.0250;
    private static final double maxRotate = 0.75;
    private static final int blueTargetId = 20;

    // Drive Constants // TODO : Edit Rotate Power To Rotate Right
    private static final double drivePower = 0.50;
    private static final double rotatePower = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Active If Using LimeLight
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 30);
        robot.vision.limeLight.setPollRateHz(90);
        robot.vision.limeLight.pipelineSwitch(0);

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("PreLoad Order: L - 2 Purple | R - 1 Green");
        telemetry.addLine("Scanning Motif...");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            robot.vision.updateMotif();

            telemetry.addLine("=== Motif Detection ===");
            telemetry.addData("Has Motif", robot.vision.hasMotif());
            telemetry.addData("Motif Tag ID", robot.vision.motifTagId);
            telemetry.addData("Motif Pattern", robot.vision.motifPattern);
            telemetry.update();

            idle();
        }

        if (isStopRequested()) return;

        // Motif Detected or BackUp
        String motif = robot.vision.hasMotif() ? robot.vision.motifPattern : "GPP";

        // Blue PipeLine
        robot.vision.limeLight.pipelineSwitch(1);

        // Sequence
        spinUpFlywheel();
        aimAtTag(blueTargetId);
        shootMotif(motif);
        robot.driveTrain.driveForwardInches(this, 12.0, drivePower);

        // Shutdown
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.driveTrain.tankDrive(0.0, 0.0);
        robot.driveTrain.brake();
        robot.vision.limeLight.stop();
    }

    private void spinUpFlywheel() {
        while (opModeIsActive()) {
            double measuredFlywheelRps1 = Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) / TicksPerRev;
            double measuredFlywheelRps2 = Math.abs(robot.scoringMechanisms.flyWheel2.getVelocity()) / TicksPerRev;
            double averageFlywheelRps = (measuredFlywheelRps1 + measuredFlywheelRps2) / 2.0;

            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            telemetry.addLine("=== Shooter ===");
            telemetry.addData("Flywheel1 RPS", "%5.2f", measuredFlywheelRps1);
            telemetry.addData("Flywheel2 RPS", "%5.2f", measuredFlywheelRps2);
            telemetry.addData("Avg RPS", "%5.2f", averageFlywheelRps);
            telemetry.addData("Target RPS", "%5.2f", targetRPS);
            telemetry.addData("Shooter Status", ((averageFlywheelRps >= (targetRPS - 1.0))) ? "READY" : "CHARGING");
            telemetry.update();

            if (averageFlywheelRps >= (targetRPS - 1.0)) {
                break;
            }

            idle();
        }
    }

    private void aimAtTag(int desiredTagId) {
        // Phase 1: Rotate to See Desired Tag
        while (opModeIsActive()) {
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();
            int tagId = -1;

            if (hasTarget) {
                java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    tagId = fiducials.get(0).getFiducialId();
                }
            }

            if (hasTarget && tagId == desiredTagId) {
                break;
            }

            // Rotate Left Till Tag In View
            robot.driveTrain.tankDrive(0.0, rotatePower);

            idle();
        }

        // Phase 2: AimBot
        while (opModeIsActive()) {
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();
            if (!hasTarget) {
                robot.driveTrain.tankDrive(0.0, 0.0);
                idle();
                continue;
            }

            double headingError = result.getTx();
            double rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

            robot.driveTrain.tankDrive(0.0, rotate);

            if (Math.abs(headingError) <= 1.0) {
                break;
            }

            idle();
        }

        // Stop After Alignment
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
            default:
                fireLeft();
                fireLeft();
                fireRight();
                break;
        }
    }

    private void fireLeft() {
        final int gateOpenIterations = 15;
        final int totalIterations = 90;

        for (int i = 0; opModeIsActive() && i < totalIterations; i++) {
            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            robot.scoringMechanisms.rollerIntake.setPower(1.0);
            robot.scoringMechanisms.sorterIntake.setPower(1.0);

            if (i < gateOpenIterations) {
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

    private void fireRight() {
        final int gateOpenIterations = 15;
        final int totalIterations = 90;

        for (int i = 0; opModeIsActive() && i < totalIterations; i++) {
            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            robot.scoringMechanisms.rollerIntake.setPower(1.0);
            robot.scoringMechanisms.sorterIntake.setPower(-1.0);

            if (i < gateOpenIterations) {
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
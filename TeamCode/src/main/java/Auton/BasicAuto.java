package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import Systems.Robot;

/**
 * Title: BasicAuto - Designed for FTC Decode 2025-26
 * Desc: Simple auton that:
 *   1) Reads an AprilTag motif with Limelight
 *   2) Rotates ~10 degrees to the left
 *   3) Shoots artifacts in the order specified by the motif
 *   4) Drives forward
 *
 * Motif tags:
 *   ID 21 -> GPP
 *   ID 22 -> PGP
 *   ID 23 -> PPG
 *
 * Left side = Purple, Right side = Green.
 *
 * @Author Gurtej Singh
 */

@Autonomous(name = "BasicAuto", group = "Auton")
public class BasicAuto extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // Flywheel constants for 6000RPM Yellow Jacket motors
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double FLYWHEEL_TARGET_RPS    = 57.50;  // desired revs/sec at shaft

    // Servo positions (must match TeleOp)
    private static final double ARTIFACT_HOLD_LEFT     = 0.0;
    private static final double ARTIFACT_RELEASE_LEFT  = 0.5;
    private static final double ARTIFACT_HOLD_RIGHT    = 0.5;
    private static final double ARTIFACT_RELEASE_RIGHT = 1.0;

    // Rotation / drive timing (tune on-field)
    private static final double ROTATE_POWER  = -0.3;   // power for rotation
    private static final long   ROTATE_TIME_MS = 275;  // ~10 degrees, tune as needed

    private static final double DRIVE_POWER   = 0.5;   // forward drive power
    private static final long   DRIVE_TIME_MS = 1000;   // forward duration, tune as needed

    // Motif variables
    private int motifID = 0;
    private String motifPattern = "UNKNOWN";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // If using Limelight, start stream to Dashboard
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 90);

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Read motif from AprilTag
        motifID = readMotifFromTag(3000); // 3s timeout
        telemetry.addData("Motif ID", motifID);
        telemetry.addData("Motif Pattern", motifPattern);
        telemetry.update();

        sleep(2000);

        // 2) Rotate 10 degrees to the left (approximate, time-based)
        rotateLeftByTime(ROTATE_POWER, ROTATE_TIME_MS);

        // 3) Shoot according to motif pattern
        shootBasedOnMotif(motifID);

        sleep(5000);

        // 4) Drive forward
        driveForwardByTime(DRIVE_POWER, DRIVE_TIME_MS);

        // Stop everything
        stopAll();
    }

    /**
     * Attempts to read an AprilTag motif within the given timeout.
     * Tag IDs:
     *   21 -> GPP
     *   22 -> PGP
     *   23 -> PPG
     */
    private int readMotifFromTag(long timeoutMs) {
        long start = System.currentTimeMillis();
        motifPattern = "UNKNOWN";
        motifID = 0;

        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            LLResult res = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = res != null && res.isValid();

            telemetry.addData("LimeLight Search", hasTarget ? "Target in View" : "Searching");
            telemetry.update();

            if (hasTarget) {
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

            idle();
        }

        // If we reach here, we didn't see a valid tag in time
        return 0; // UNKNOWN motif
    }

    /**
     * Rotate left by commanding a turn for a fixed time.
     * Uses tankDrive(drive, rotate) with drive=0.
     */
    private void rotateLeftByTime(double power, long durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < durationMs) {
            // Positive rotate here should be tuned to turn left; adjust sign if needed.
            robot.driveTrain.tankDrive(0.0, power);
            idle();
        }
        robot.driveTrain.tankDrive(0.0, 0.0);
    }

    /**
     * Drives forward for a fixed time.
     */
    private void driveForwardByTime(double power, long durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < durationMs) {
            robot.driveTrain.tankDrive(power, 0.0);
            idle();
        }
        robot.driveTrain.tankDrive(0.0, 0.0);
    }

    /**
     * Spin up flywheel to target speed using setVelocity and wait until it's ready
     * or until a timeout.
     */
    private void chargeFlywheel() {
        double targetTicksPerSec = FLYWHEEL_TARGET_RPS * FLYWHEEL_TICKS_PER_REV; // negative for correct direction

        robot.scoringMechanisms.flyWheel1.setVelocity(targetTicksPerSec);
        robot.scoringMechanisms.flyWheel2.setVelocity(targetTicksPerSec);

        long spinStart = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - spinStart) < 2000) {
            double measuredRps = Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) / FLYWHEEL_TICKS_PER_REV;

            telemetry.addData("Flywheel RPS", measuredRps);
            telemetry.addData("Target RPS", FLYWHEEL_TARGET_RPS);
            telemetry.update();

            if (measuredRps >= (FLYWHEEL_TARGET_RPS - 0.5)) break;
            idle();
        }
    }

    /**
     * Shoots artifacts according to the motif pattern.
     * ID 21 -> GPP (Green, Purple, Purple)
     * ID 22 -> PGP (Purple, Green, Purple)
     * ID 23 -> PPG (Purple, Purple, Green)
     */
    private void shootBasedOnMotif(int motif) {
        // Spin up first
        chargeFlywheel();

        switch (motif) {
            case 21: // GPP
                // Shot 1: Green (right)
                fireRightOnce();
                // Shot 2: Purple (left)
                fireLeftOnce();
                // Shot 3: Purple (left)
                fireLeftOnce();
                break;

            case 22: // PGP
                // Shot 1: Purple (left)
                fireLeftOnce();
                // Shot 2: Green (right)
                fireRightOnce();
                // Shot 3: Purple (left)
                fireLeftOnce();
                break;

            case 23: // PPG
                // Shot 1: Purple (left)
                fireLeftOnce();
                // Shot 2: Purple (left)
                fireLeftOnce();
                // Shot 3: Green (right)
                fireRightOnce();
                break;

            default:
                // Unknown motif: fire both sides together for 2s
                robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_RELEASE_LEFT);
                robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_RELEASE_RIGHT);
                sleep(2000);
                robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_HOLD_LEFT);
                robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_HOLD_RIGHT);
                break;
        }

        // Stop flywheel after shooting
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
    }


    private void fireLeftOnce() {
        // Release one artifact on the left (purple) side
        robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_RELEASE_LEFT);
        sleep(500); // tune open time for a single artifact
        robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_HOLD_LEFT);
        sleep(250); // brief settle before feeding next artifact

        // After the shot, run intake + sorter to feed the next artifact
        // Left side uses the same direction as TeleOp left-bumper behavior: roller +1, sorter +1
        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(1.0);
        sleep(1000); // tune duration as needed to advance exactly one artifact
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }

    private void fireRightOnce() {
        // Release one artifact on the right (green) side
        robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_RELEASE_RIGHT);
        sleep(500); // tune open time for a single artifact
        robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_HOLD_RIGHT);
        sleep(250); // brief settle before feeding next artifact

        // After the shot, run intake + sorter to feed the next artifact
        // Right side uses the same direction as TeleOp right-bumper behavior: roller +1, sorter -1
        robot.scoringMechanisms.rollerIntake.setPower(1.0);
        robot.scoringMechanisms.sorterIntake.setPower(-1.0);
        sleep(1000); // tune duration as needed to advance exactly one artifact
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
    }

    private void stopAll() {
        robot.driveTrain.tankDrive(0.0, 0.0);
        robot.scoringMechanisms.flyWheel1.setPower(0.0);
        robot.scoringMechanisms.flyWheel2.setPower(0.0);
        robot.scoringMechanisms.rollerIntake.setPower(0.0);
        robot.scoringMechanisms.sorterIntake.setPower(0.0);
        robot.scoringMechanisms.leftRelease.setPosition(ARTIFACT_HOLD_LEFT);
        robot.scoringMechanisms.rightRelease.setPosition(ARTIFACT_HOLD_RIGHT);
        try {
            robot.vision.limeLight.stop();
        } catch (Exception ignored) {
        }
    }
}
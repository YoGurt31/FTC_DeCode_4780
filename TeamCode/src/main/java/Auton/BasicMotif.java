package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import Systems.Robot;

/**
 * Title: BasicMotif - Designed for FTC Decode 2025-26
 * Desc: Basic Auton for a 4-Motor Tank DriveTrain
 *
 * @Author Gurej Singh
 */

@Autonomous(name = "BasicMotif", group = "Auton")
public class BasicMotif extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Active If Using WebCam
        // FtcDashboard.getInstance().startCameraStream(robot.vision.visionPortal, 20);

        // Active If Using LimeLight
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 30);

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        // Wait for the Start button to be pressed on the Driver Station.
        waitForStart();

        while (opModeIsActive()) {

            // 1) Drive Forward 24 Inches
            robot.driveTrain.driveStraight(this, 24.0, 0.75);

            // 2) Turn 180 Degrees
            robot.driveTrain.turnToHeading(this, 180.0, 0.35);

            // 3) Search For AprilTag
            LLResult targetResult = null;
            long searchStart = System.currentTimeMillis();

            while (opModeIsActive() && (System.currentTimeMillis() - searchStart) < 3000) {
                targetResult = robot.vision.limeLight.getLatestResult();
                boolean hasTarget = targetResult != null && targetResult.isValid();

                telemetry.addData("LimeLight Search", hasTarget ? "Target in View" : "Searching");
                if (hasTarget) break;

                telemetry.update();
                idle();
            }

            // 4) If Tag Found, Align To It
            if (opModeIsActive() && targetResult != null && targetResult.isValid()) {
                // TODO: Tune Values
                double desiredRange = 0.0; // TY VALUE READ @ SPECIFIC POSITION - For 24 In, Read TY Value When Robot Is 24 In From Target
                double driveGain = 0.020;
                double rotateGain = 0.010;
                double maxDrive = 0.50;
                double maxRotate = 0.25;

                long alignStart = System.currentTimeMillis();
                while (opModeIsActive() && (System.currentTimeMillis() - alignStart) < 3000) {
                    LLResult current = robot.vision.limeLight.getLatestResult();
                    if (current == null || !current.isValid()) break;

                    double rangeError    = targetResult.getTy() - desiredRange;
                    double headingError  = targetResult.getTx();

                    double driveCmd = Range.clip(rangeError * driveGain, -maxDrive, maxDrive);
                    double rotateCmd = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

                    robot.driveTrain.tankDrive(driveCmd, rotateCmd);

                    telemetry.addData("Align", "Drive %.2f, Turn %.2f", driveCmd, rotateCmd);
                    telemetry.addData("Range Error", targetResult.getTy());
                    telemetry.addData("Heading Error", targetResult.getTx());
                    telemetry.update();
                    idle();
                }

                // Stop after alignment
                robot.driveTrain.tankDrive(0, 0);
            }

            // 5) Spin up flywheel and shoot artifacts
            final double targetRPS = 25.0;
            final double TicksPerRev = 4000;
            final double artifactHold = 1.0;
            final double artifactRelease = 0.0;

            robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
            robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);

            long spinStart = System.currentTimeMillis();
            double measuredRPS = 0.0;
            while (opModeIsActive() && (System.currentTimeMillis() - spinStart) < 2000) {
                measuredRPS = robot.scoringMechanisms.flyWheel1.getVelocity() / TicksPerRev;
                telemetry.addData("Flywheel RPS", measuredRPS);
                telemetry.update();
                if (measuredRPS >= (targetRPS - 0.5)) break;
                idle();
            }

            // Release Artifacts
            robot.scoringMechanisms.leftRelease.setPosition(artifactRelease);
            robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
            sleep(500);

            // Stop
            robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
            robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
            robot.scoringMechanisms.flyWheel1.setPower(0);
            robot.scoringMechanisms.flyWheel2.setPower(0);

            telemetry.update();
        }
        robot.vision.limeLight.stop();
    }
}
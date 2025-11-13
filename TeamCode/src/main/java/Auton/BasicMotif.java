package Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import Systems.Robot;

/**
 * Title: BasicMotif - Designed for FTC Decode 2025-26
 * Desc: Basic Auton for a 4-Motor Tank DriveTrain
 *
 * @Author Gurej Singh
 * @Version 1.0
 */

@Autonomous(name = "BasicMotif", group = "Auton")
public class BasicMotif extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(robot.vision.visionPortal, 20);

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
            AprilTagDetection targetTag = null;
            long searchStart = System.currentTimeMillis();
            while (opModeIsActive() && (System.currentTimeMillis() - searchStart) < 3000 && targetTag == null) {
                List<AprilTagDetection> detections = robot.vision.aprilTag.getDetections();
                for (AprilTagDetection det : detections) {
                    if (det.metadata != null) {
                        targetTag = det;
                        break;
                    }
                }
                telemetry.addData("Tag Search", targetTag == null ? "Searching" : ("Found ID " + targetTag.id));
                telemetry.update();
                idle();
            }

            // 4) If Tag Found, Align To It
            if (opModeIsActive() && targetTag != null) {
                // TODO: Tune Values
                double desiredRange = 12.0;
                double driveGain = 0.020;
                double rotateGain = 0.010;
                double maxDrive = 0.50;
                double maxRotate = 0.25;

                long alignStart = System.currentTimeMillis();
                while (opModeIsActive() && (System.currentTimeMillis() - alignStart) < 5000) {
                    List<AprilTagDetection> detections = robot.vision.aprilTag.getDetections();
                    AprilTagDetection current = null;
                    for (AprilTagDetection det : detections) {
                        if (det.id == targetTag.id) {
                            current = det;
                            break;
                        }
                    }
                    if (current == null || current.metadata == null) break;

                    double rangeError = current.ftcPose.range - desiredRange;
                    double bearingError = current.ftcPose.bearing; // degrees

                    double driveCmd = Range.clip(rangeError * driveGain, -maxDrive, maxDrive);
                    double rotateCmd = Range.clip(bearingError * rotateGain, -maxRotate, maxRotate);

                    robot.driveTrain.tankDrive(driveCmd, rotateCmd);

                    telemetry.addData("Align ID", current.id);
                    telemetry.addData("Range", "%5.1f", current.ftcPose.range);
                    telemetry.addData("Bearing", "%5.1f", current.ftcPose.bearing);
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
        }
    }
}
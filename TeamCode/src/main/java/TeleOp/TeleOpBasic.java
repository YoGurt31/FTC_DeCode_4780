package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import Systems.Robot;

/**
 * Title: TeleOpBasic - Designed for FTC Decode 2025-26
 * Desc: Basic TeleOp for a 4-Motor Tank DriveTrain
 * Includes Driving and Rotating
 *
 * Controls (GamePad1):
 * - Left Analog X:      N/A
 * - Left Analog Y:      Drives Robot Forward and Backwards
 * - Right Analog X:     Rotates Robot Left and Right
 * - Right Analog Y:     N/A
 * - Left Bumper:        Activate Intake + Sort to Left Side
 * - Left Trigger:       Shoot Left Side
 * - Right Bumper:       Activate Intake + Sort to Right Side
 * - Right Trigger:      Shoot Right Side
 * - DPad Up:            Returns Function
 * - DPad Down:          Returns Function
 * - DPad Left:          Returns Function
 * - DPad Right:         Returns Function
 * - FaceButton Up:      Returns Function
 * - FaceButton Down:    Returns Function
 * - FaceButton Left:    Returns Function
 * - FaceButton Right:   Returns Function
 *
 * @Author Gurej Singh
 * @Version 1.0
 */

@TeleOp(name = "Tank", group = "TeleOp")
public class TeleOpBasic extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // FlyWheel Variables
    private boolean flyWheelOn = false;
    private static final double targetRPS = 25.0;
    private static final double TicksPerRev = 4000; // FlyWheel ELC Encoder Resolution
    private final double artifactHold = 1.0;
    private final double artifactRelease = 0.0;

    // AprilTag / Vision Variables
    // TODO: Tune Values
    final double distance = 12.0; // Distance To Reach From AprilTag (In Inches)

    final double driveGain  =  0.020;
    final double rotateGain =  0.010;

    final double maxDrive  = 0.50;
    final double maxRotate = 0.25;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(robot.vision.visionPortal, 20);

        double drive = 0, rotate = 0;
        boolean targetFound = false;

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        // Wait for the Start button to be pressed on the Driver Station.
        waitForStart();

        while (opModeIsActive()) {

            // AprilTag Targeting
            boolean activeTargeting = gamepad1.left_bumper;

            List<AprilTagDetection> currentDetections = robot.vision.aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == -1) { // -1 Refers to ANY Tag
                        targetFound = true;
                        robot.vision.desiredTag = detection;
                        break;
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", robot.vision.desiredTag.id, robot.vision.desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", robot.vision.desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", robot.vision.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", robot.vision.desiredTag.ftcPose.yaw);
            }

            if (activeTargeting && targetFound) {
                double rangeError    = (robot.vision.desiredTag.ftcPose.range - distance);
                double headingError  = (robot.vision.desiredTag.ftcPose.bearing);

                drive  = Range.clip(rangeError * driveGain, -maxDrive, maxDrive);
                rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f ", drive, rotate);
            } else {
                drive  = -gamepad1.right_stick_y;
                rotate = -gamepad1.left_stick_x;

                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f ", drive, rotate);
            }

            robot.driveTrain.tankDrive(drive, rotate);

            // Brake DriveTrain if No Joystick Input (1e-2 (0.01) Accounts for Stick Drift)
            if (Math.abs(drive) <= 1e-2 && Math.abs(rotate) <= 1e-2) {
                robot.driveTrain.brake();
            }

            // Intake Control
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.scoringMechanisms.rollerIntake.setPower(-1.0);
                robot.scoringMechanisms.sorterIntake.setPower(0.0);
            } else if (gamepad1.left_bumper) {
                robot.scoringMechanisms.rollerIntake.setPower(1);
                robot.scoringMechanisms.sorterIntake.setPower(1);
            } else if (gamepad1.right_bumper) {
                robot.scoringMechanisms.rollerIntake.setPower(1);
                robot.scoringMechanisms.sorterIntake.setPower(-1);
            } else {
                robot.scoringMechanisms.rollerIntake.setPower(0.0);
                robot.scoringMechanisms.sorterIntake.setPower(0.0);
            }

            // FlyWheel Control
            double measuredFlywheelRps = (robot.scoringMechanisms.flyWheel1.getVelocity() / TicksPerRev);

            if (gamepad1.left_trigger >= 0.05 || gamepad1.right_trigger >= 0.05) {
                robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
                robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);
            } else {
                robot.scoringMechanisms.flyWheel1.setPower(0);
                robot.scoringMechanisms.flyWheel2.setPower(0);
                robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
            }

            if (gamepad1.left_trigger >= 0.05) {
                if (measuredFlywheelRps >= targetRPS - 0.5) {
                    robot.scoringMechanisms.leftRelease.setPosition(artifactRelease);
                } else {
                    robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
                }
            } else {
                robot.scoringMechanisms.leftRelease.setPosition(artifactHold);
            }

            if (gamepad1.right_trigger >= 0.05) {
                if (measuredFlywheelRps >= targetRPS - 0.5) {
                    robot.scoringMechanisms.rightRelease.setPosition(artifactRelease);
                } else {
                    robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
                }
            } else {
                robot.scoringMechanisms.rightRelease.setPosition(artifactHold);
            }

            telemetry.addData("Flywheel RPS (Measured)", measuredFlywheelRps);
            telemetry.addData("Flywheel RPS (Target)", targetRPS);
            telemetry.addData("Left Release", ((robot.scoringMechanisms.leftRelease.getPosition()  <= 0.05) ? "Open" : "Closed"));
            telemetry.addData("Right Release", ((robot.scoringMechanisms.rightRelease.getPosition()  <= 0.05) ? "Open" : "Closed"));
        }
    }
}
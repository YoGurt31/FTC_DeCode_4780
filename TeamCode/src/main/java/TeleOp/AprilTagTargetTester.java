package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;

import Systems.Robot;

/**
 * Title: TeleOpBasic - Designed for FTC Decode 2025-26
 * Desc: Basic TeleOp for a 4-Motor Mecanum DriveTrain
 * Includes Driving, Strafing, and Rotating
 *
 * Controls (GamePad1):
 * - Left Analog X:      Strafes Robot Left and Right
 * - Left Analog Y:      Drives Robot Forward and Backwards
 * - Right Analog X:     Rotates Robot Left and Right
 * - Right Analog Y:     N/A
 * - Left Bumper:        Returns Function
 * - Left Trigger:       Returns Function
 * - Right Bumper:       Starts & Stops (Toggles) Fly Wheel
 * - Right Trigger:      Returns Function
 * - DPad Up:            Increases FlyWheel RPS by 1
 * - DPad Down:          Decreases FlyWheel RPS by 1
 * - DPad Left:          Returns Function
 * - DPad Right:         Returns Function
 * - FaceButton Up:      Returns Function
 * - FaceButton Down:    Returns Function
 * - FaceButton Left:    Returns Function
 * - FaceButton Right:   Returns Function
 *
 * @Author Gurtej Singh
 * @Version 1.0
 */

@TeleOp(name = "AprilTag Test", group = "TeleOp")
@Disabled
public class AprilTagTargetTester extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // FlyWheel Variables
    private boolean flyWheelOn = false;
    private double RPS = 5;
    private double gearRatioMotorToFlyWheel = 0.2;
    private double gearRatioFlyWheelToMotor = 5.0;


    // AprilTag / Vision
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // -1 = Any Tag (Find Specific Tag)

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Gains (from RobotAutoDriveToAprilTagOmni sample)
    private static final double SPEED_GAIN  = 0.02;   // forward speed gain (range)
    private static final double STRAFE_GAIN = 0.015;  // strafe gain (yaw)
    private static final double TURN_GAIN   = 0.01;   // turn gain (bearing)

    private static final double MAX_AUTO_SPEED  = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN   = 0.25;

    // AprilTag Vision Initialization
    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    // Linear Mapping for RPS based on Range
    private double rpsFromRange(double rangeInches) {
        double NEAR_IN = 6.0;  // If  6 Inches away from AprilTag
        double FAR_IN  = 72.0; // If 72 Inches away from AprilTag
        double i = (rangeInches - NEAR_IN) / (FAR_IN - NEAR_IN);
        i = Range.clip(i, 0.0, 1.0);
        return 5.0 + i * (50.0 - 5.0);
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        initAprilTag();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        // Wait for the Start button to be pressed on the Driver Station.
        waitForStart();

        while (opModeIsActive()) {

            double drive   = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

            double frontLeftPower  = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower   = drive - strafe + rotate;
            double backRightPower  = drive + strafe - rotate;

            // Prevents Motors from Exceeding 100% Power
            double[] powers = { frontLeftPower, frontRightPower, backLeftPower, backRightPower };
            double maxPower = 0.0;
            for (double pow : powers) {
                maxPower = Math.max(maxPower, Math.abs(pow));
            }

            if (maxPower > 1.0) {
                frontLeftPower  /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower   /= maxPower;
                backRightPower  /= maxPower;
            }

            robot.driveTrain.mecDrive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            if (drive == 0 && strafe == 0) {
                robot.driveTrain.brake();
            }

            // Fly Wheel Manual Control
            if (gamepad1.rightBumperWasPressed()) {
                flyWheelOn = !flyWheelOn;
            }

            if (gamepad1.dpadUpWasPressed() && (RPS < 50.0)) {
                RPS += 1.0;
            }

            if (gamepad1.dpadDownWasPressed() && (RPS > 0.0)) {
                RPS -= 1.0;
            }

            // AprilTag Assisted Control
            boolean targeting = gamepad1.left_bumper;
            boolean targetFound = false;
            AprilTagDetection desired = null;

            if (targeting) {
                // Look for Desired AprilTag
                for (AprilTagDetection det : aprilTag.getDetections()) {
                    if (det.metadata != null && (DESIRED_TAG_ID < 0 || det.id == DESIRED_TAG_ID)) {
                        desired = det; targetFound = true; break;
                    }
                }

                if (targetFound) {
                    // Compute Auto Drive Commands from Position
                    double rangeError   = desired.ftcPose.range;   // Inches from Camera
                    double bearingError = desired.ftcPose.bearing; // Deg Left/Right
                    double yawError     = desired.ftcPose.yaw;     // Deg Tag Rotation

                    double driveCmd  = Range.clip(rangeError   * SPEED_GAIN,  -MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
                    double strafeCmd = Range.clip(-yawError    * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    double turnCmd   = Range.clip(bearingError * TURN_GAIN,   -MAX_AUTO_TURN,   MAX_AUTO_TURN);

                    double fl = driveCmd + strafeCmd + turnCmd;
                    double fr = driveCmd - strafeCmd - turnCmd;
                    double bl = driveCmd - strafeCmd + turnCmd;
                    double br = driveCmd + strafeCmd - turnCmd;

                    // Normalize Wheel Powers
                    double[] autoPowers = { fl, fr, bl, br };
                    double maxAutoPower = 0.0;
                    for (double autoPow : autoPowers) {
                        maxAutoPower = Math.max(maxAutoPower, Math.abs(autoPow));
                    }

                    if (maxAutoPower > 1.0) {
                        fl /= maxAutoPower;
                        fr /= maxAutoPower;
                        bl /= maxAutoPower;
                        br /= maxAutoPower;
                    }

                    robot.driveTrain.mecDrive(fl, fr, bl, br);

                    // Set FlyWHeel RPS from Distance
                    RPS = rpsFromRange(rangeError);

                    // Telemetry for Targeting
                    telemetry.addLine("MODE: Targeting AprilTag (hold LB)");
                    telemetry.addData("Tag", "ID %d %s", desired.id, desired.metadata.name);
                    telemetry.addData("Range (in)", "%.1f", desired.ftcPose.range);
                    telemetry.addData("Bearing (deg)", "%.1f", desired.ftcPose.bearing);
                    telemetry.addData("Yaw (deg)", "%.1f", desired.ftcPose.yaw);
                } else {
                    telemetry.addLine("MODE: Searching - No Tag Visible");
                }
            } else {
                telemetry.addLine("MODE: Manual");
            }

            // Fly Wheel Data
            double targetFlywheelRps = flyWheelOn ? RPS : 0.0;
            double measuredFlywheelRps = (robot.scoringMechanisms.flyWheel.getVelocity(AngleUnit.DEGREES) / 360.0) * gearRatioFlyWheelToMotor;
            robot.scoringMechanisms.flyWheel.setVelocity((targetFlywheelRps * 360.0 * gearRatioMotorToFlyWheel), AngleUnit.DEGREES);

            telemetry.addLine();
            telemetry.addData("Flywheel", flyWheelOn ? "(ON)" : "(OFF)");
            telemetry.addData("Target Velocity (RPS)", String.format("%.1f", targetFlywheelRps));
            telemetry.addData("Measured Velocity (RPS)", String.format("%.1f", measuredFlywheelRps));
            telemetry.update();

        }

        if (visionPortal != null) {
            try { visionPortal.close(); } catch (Exception ignored) {}
        }
    }
}
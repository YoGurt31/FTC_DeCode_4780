package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Systems.Robot;

/**
 * Title: TeleOpBasic - Designed for FTC Decode 2025-26
 * Desc: Basic TeleOp for a 4-Motor Tank DriveTrain
 * Includes Driving and Rotating
 * <p>
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
 */

@TeleOp(name = "Tank", group = "TeleOp")
public class TeleOpBasic extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // FlyWheel Variables
    private static final double targetRPS = 57.5;
    private static final double TicksPerRev = 28.0; // FlyWheel Encoder Resolution
    private final double artifactHoldRight = 0.5;
    private final double artifactHoldLeft = 0.0;
    private final double artifactReleaseRight = 1.0;
    private final double artifactReleaseLeft = 0.5;
    private long leftShotEndTime = 0;
    private long rightShotEndTime = 0;
    private long leftGateOpenUntil = 0;
    private long rightGateOpenUntil = 0;


    // AprilTag / Vision Variables
    private static final double rotateGain = 0.0250;
    private static final double maxRotate = 0.75;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Active If Using WebCam
        // FtcDashboard.getInstance().startCameraStream(robot.vision.visionPortal, 20);

        // Active If Using LimeLight
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 30);

        double drive = 0, rotate = 0;

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        // Wait for the Start button to be pressed on the Driver Station.
        waitForStart();

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();

            // AprilTag Targeting
            boolean activeTargeting = gamepad1.left_trigger >= 0.5;
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            if (activeTargeting && hasTarget) {
                double headingError = result.getTx();

                drive = -gamepad1.left_stick_y;
                rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);

                telemetry.addData("Limelight", "Tracking");
                telemetry.addData("Heading Error", result.getTx());
                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f ", drive, rotate);
            } else {
                drive = -gamepad1.left_stick_y;
                rotate = gamepad1.right_stick_x;

                telemetry.addData("Limelight", hasTarget ? "Target in View" : "No Target");
                telemetry.addData("Manual", "Drive %5.2f, Turn %5.2f ", drive, rotate);
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
            double measuredFlywheelRps1 = (Math.abs(robot.scoringMechanisms.flyWheel1.getVelocity()) / TicksPerRev);
            double measuredFlywheelRps2 = (Math.abs(robot.scoringMechanisms.flyWheel2.getVelocity()) / TicksPerRev);

            if (gamepad1.right_trigger >= 0.05) {
                robot.scoringMechanisms.flyWheel1.setVelocity(targetRPS * TicksPerRev);
                robot.scoringMechanisms.flyWheel2.setVelocity(targetRPS * TicksPerRev);
            } else {
                robot.scoringMechanisms.flyWheel1.setPower(0);
                robot.scoringMechanisms.flyWheel2.setPower(0);
            }

            // Artifact Release Control
            if (gamepad1.yWasPressed()) {
                leftGateOpenUntil = now + 250;
                leftShotEndTime = now + 1500;
            }

            if (gamepad1.bWasPressed()) {
                rightGateOpenUntil = now + 250;
                rightShotEndTime = now + 1500;
            }

            boolean leftShotActive = now < leftShotEndTime;
            boolean rightShotActive = now < rightShotEndTime;
            boolean anyShotActive = leftShotActive || rightShotActive;

            if (anyShotActive) {
                robot.scoringMechanisms.rollerIntake.setPower(1.0);

                if (leftShotActive && !rightShotActive) {
                    robot.scoringMechanisms.sorterIntake.setPower(1.0);
                } else if (rightShotActive && !leftShotActive) {
                    robot.scoringMechanisms.sorterIntake.setPower(-1.0);
                } else {
                    robot.scoringMechanisms.sorterIntake.setPower(0.0);
                }
            }

            if (now < leftGateOpenUntil) {
                robot.scoringMechanisms.leftRelease.setPosition(artifactReleaseLeft);
            } else {
                robot.scoringMechanisms.leftRelease.setPosition(artifactHoldLeft);
            }

            if (now < rightGateOpenUntil) {
                robot.scoringMechanisms.rightRelease.setPosition(artifactReleaseRight);
            } else {
                robot.scoringMechanisms.rightRelease.setPosition(artifactHoldRight);
            }

            telemetry.addData("Flywheel1 Vel", robot.scoringMechanisms.flyWheel1.getVelocity());
            telemetry.addData("Flywheel2 Vel", robot.scoringMechanisms.flyWheel2.getVelocity());
            telemetry.addData("Flywheel1 RPS (Measured)", measuredFlywheelRps1);
            telemetry.addData("Flywheel2 RPS (Measured)", measuredFlywheelRps2);
            telemetry.addData("Flywheel RPS (Target)", targetRPS);

            telemetry.update();
        }
        robot.vision.limeLight.stop();
    }
}
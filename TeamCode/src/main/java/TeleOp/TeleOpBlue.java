package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * - Left Trigger:       AimBot
 * - Right Bumper:       Activate Intake + Sort to Right Side
 * - Right Trigger:      Charge FlyWheel
 * - DPad Up:            Returns Function
 * - DPad Down:          Returns Function
 * - DPad Left:          Returns Function
 * - DPad Right:         Returns Function
 * - FaceButton Up:      Shoot Left Side
 * - FaceButton Down:    Returns Function
 * - FaceButton Left:    Returns Function
 * - FaceButton Right:   Shoot Right Side
 *
 * @Author Gurtej Singh
 */

@TeleOp(name = "BLUE", group = "TeleOp")
@Disabled
public class TeleOpBlue extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    // FlyWheel Variables
    private static final double targetRPS = 53.5;
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

        // Active If Using LimeLight
        FtcDashboard.getInstance().startCameraStream(robot.vision.limeLight, 30);
        robot.vision.limeLight.setPollRateHz(90);
        robot.vision.limeLight.pipelineSwitch(2);

        double drive = 0, rotate = 0;

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();

            // AprilTag Targeting
            boolean activeTargeting = gamepad1.left_trigger >= 0.25;
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = result != null && result.isValid();

            if (activeTargeting && hasTarget) {
                double headingError = result.getTx();
                drive = -gamepad1.left_stick_y;
                rotate = Range.clip(headingError * rotateGain, -maxRotate, maxRotate);
            } else {
                drive = -gamepad1.left_stick_y;
                rotate = gamepad1.right_stick_x;
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
            if (gamepad1.xWasPressed()) {
                leftGateOpenUntil = now + 250;
                leftShotEndTime = now + 1500;
            }

            if (gamepad1.bWasPressed()) {
                rightGateOpenUntil = now + 250;
                rightShotEndTime = now + 1500;
            }

            boolean leftShotActive = now < leftShotEndTime;
            boolean rightShotActive = now < rightShotEndTime;

            if (leftShotActive || rightShotActive) {
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

            // Drive / AimBot
            telemetry.addLine("=== Drive + AimBot ===");
            telemetry.addData("Drive", "%5.2f", drive);
            telemetry.addData("Rotate", "%5.2f", rotate);
            telemetry.addData("AimBot Active", activeTargeting && hasTarget);
            telemetry.addData("Tag In View", hasTarget);
            telemetry.addLine();

            // Intake / Sorter
            telemetry.addLine("=== Intake + Sorter ===");
            telemetry.addData("Roller Status", robot.scoringMechanisms.rollerIntake.getPower() > 0 ? "ACTIVE" : "IDLE");
            String sorterStatus; if (gamepad1.left_bumper && !gamepad1.right_bumper) { sorterStatus = "LEFT"; } else if (gamepad1.right_bumper && !gamepad1.left_bumper) { sorterStatus = "RIGHT"; } else { sorterStatus = "IDLE"; }
            telemetry.addData("Sorting", sorterStatus);
            telemetry.addLine();

            // Shooter / Gates
            double averageFlywheelRps = (measuredFlywheelRps1 + measuredFlywheelRps2) / 2.0;
            telemetry.addLine("=== Shooter + Gates ===");
            telemetry.addData("Flywheel1 RPS", "%5.2f", measuredFlywheelRps1);
            telemetry.addData("Flywheel2 RPS", "%5.2f", measuredFlywheelRps2);
            telemetry.addData("Avg RPS", "%5.2f", averageFlywheelRps);
            telemetry.addData("Target RPS", "%5.2f", targetRPS);
            telemetry.addData("Shooter Status", ((averageFlywheelRps >= (targetRPS - 1.0)) && (gamepad1.right_trigger >= 0.05)) ? "READY" : "CHARGING");
            telemetry.addData("Left Gate", (now < leftGateOpenUntil) ? "Open" : "Closed");
            telemetry.addData("Right Gate", (now < rightGateOpenUntil) ? "Open" : "Closed");

            telemetry.update();
        }
        robot.vision.limeLight.stop();
    }
}
package TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import Systems.Robot;

@TeleOp(name = "BLUE", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.vision.limeLight.setPollRateHz(15);
        robot.vision.setPipeline(2);

        double drive, rotate;

        waitForStart();

        while (opModeIsActive()) {

            // AprilTag Targeting
            boolean activeTargeting = gamepad1.left_trigger >= 0.25;
            LLResult result = robot.vision.limeLight.getLatestResult();
            boolean hasTarget = false;
            double tagArea;

            if (result != null && result.isValid()) {
                java.util.List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                hasTarget = tags != null && !tags.isEmpty();
            }

            if (hasTarget) {
                tagArea = result.getTa();
                if (tagArea >= robot.vision.tagAreaThreshold) {
                    robot.scoringMechanisms.targetRPS = robot.scoringMechanisms.closeTargetRPS;
                } else {
                    robot.scoringMechanisms.targetRPS = robot.scoringMechanisms.farTargetRPS;
                }
            } else {
                robot.scoringMechanisms.targetRPS = robot.scoringMechanisms.farTargetRPS;
            }

            // AIMBOT
            if (activeTargeting && hasTarget) {
                double headingError = result.getTx();
                drive = -gamepad1.left_stick_y;
                rotate = Range.clip(headingError * robot.vision.rotateGain, -robot.vision.maxRotate, robot.vision.maxRotate);
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
            if (gamepad1.right_trigger >= 0.05) {
                robot.scoringMechanisms.flyWheel1.setVelocity(robot.scoringMechanisms.targetRPS * robot.scoringMechanisms.TicksPerRev);
                robot.scoringMechanisms.flyWheel2.setVelocity(robot.scoringMechanisms.targetRPS * robot.scoringMechanisms.TicksPerRev);
            } else {
                robot.scoringMechanisms.flyWheel1.setPower(0);
                robot.scoringMechanisms.flyWheel2.setPower(0);
            }

            // Artifact Release Control
            long now = System.currentTimeMillis();

            if (gamepad1.x) {
                robot.scoringMechanisms.leftGateOpenUntil = Long.MAX_VALUE;
                robot.scoringMechanisms.leftShotEndTime = 0;
            } else if (gamepad1.xWasReleased()) {
                robot.scoringMechanisms.leftGateOpenUntil = now + 100;
                robot.scoringMechanisms.leftShotEndTime = now + 1500;
            }

            if (gamepad1.b) {
                robot.scoringMechanisms.rightGateOpenUntil = Long.MAX_VALUE;
                robot.scoringMechanisms.rightShotEndTime = 0;
            } else if (gamepad1.bWasReleased()) {
                robot.scoringMechanisms.rightGateOpenUntil = now + 100;
                robot.scoringMechanisms.rightShotEndTime = now + 1500;
            }

            boolean leftShotActive = now < robot.scoringMechanisms.leftShotEndTime;
            boolean rightShotActive = now < robot.scoringMechanisms.rightShotEndTime;

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

            robot.scoringMechanisms.leftRelease.setPosition(now < robot.scoringMechanisms.leftGateOpenUntil ? robot.scoringMechanisms.artifactReleaseLeft : robot.scoringMechanisms.artifactHoldLeft);
            robot.scoringMechanisms.rightRelease.setPosition(now < robot.scoringMechanisms.rightGateOpenUntil ? robot.scoringMechanisms.artifactReleaseRight : robot.scoringMechanisms.artifactHoldRight);
        }
        robot.vision.limeLight.stop();
    }
}
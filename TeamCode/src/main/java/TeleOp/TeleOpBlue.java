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

    // FlyWheel Variables
    private static double targetRPS = 0.0;
    private static final double farTargetRPS = 54.5;
    private static final double closeTargetRPS = 48.5;
    private static final double TicksPerRev = 28.0;
    private final double artifactHoldRight = 0.5;
    private final double artifactHoldLeft = 0.0;
    private final double artifactReleaseRight = 1.0;
    private final double artifactReleaseLeft = 0.5;
    private long leftShotEndTime = 0;
    private long rightShotEndTime = 0;
    private long leftGateOpenUntil = 0;
    private long rightGateOpenUntil = 0;

    // AprilTag / Vision Variables
    private static final int PipeLine = 2;
    private static final double rotateGain = 0.0250;
    private static final double maxRotate = 0.75;
    private static final double tagAreaThreshold = 0.8;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.vision.limeLight.setPollRateHz(15);
        robot.vision.setPipeline(PipeLine);

        double drive, rotate;

        waitForStart();

        while (opModeIsActive()) {

            // LimeLight FailSafe
            robot.vision.maintainPipeline();
            if (gamepad1.leftStickButtonWasPressed() && gamepad1.rightStickButtonWasPressed()) {
                robot.vision.reboot();
            }

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
                if (tagArea >= tagAreaThreshold) {
                    targetRPS = closeTargetRPS;
                } else {
                    targetRPS = farTargetRPS;
                }
            } else {
                targetRPS = farTargetRPS;
            }

            // AIMBOT
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
            long now = System.currentTimeMillis();

            if (gamepad1.x) {
                leftGateOpenUntil = Long.MAX_VALUE;
                leftShotEndTime = 0;
            } else if (gamepad1.xWasReleased()) {
                leftGateOpenUntil = now + 100;
                leftShotEndTime = now + 1500;
            }

            if (gamepad1.b) {
                rightGateOpenUntil = Long.MAX_VALUE;
                rightShotEndTime = 0;
            } else if (gamepad1.bWasReleased()) {
                rightGateOpenUntil = now + 100;
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

            robot.scoringMechanisms.leftRelease.setPosition(now < leftGateOpenUntil ? artifactReleaseLeft : artifactHoldLeft);
            robot.scoringMechanisms.rightRelease.setPosition(now < rightGateOpenUntil ? artifactReleaseRight : artifactHoldRight);
        }
        robot.vision.limeLight.stop();
    }
}
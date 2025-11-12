package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Systems.Robot;

/**
 * Title: TeleOpBasic - Designed for FTC Decode 2025-26
 * Desc: Basic TeleOp for a 4-Motor Mecanum DriveTrain
 * Includes Driving, Strafing, and Rotating
 *
 * Controls (GamePad1):
 * - Left Analog X:      N/A
 * - Left Analog Y:      Drives Robot Forward and Backwards
 * - Right Analog X:     Rotates Robot Left and Right
 * - Right Analog Y:     N/A
 * - Left Bumper:        Returns Function
 * - Left Trigger:       Returns Function
 * - Right Bumper:       Starts & Stops (Toggles) Fly Wheels
 * - Right Trigger:      Activates Roller Intake
 * - DPad Up:            Increases FlyWheel RPS by 1
 * - DPad Down:          Decreases FlyWheel RPS by 1
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

@TeleOp(name = "Basic Tank", group = "TeleOp")
public class TeleOpBasic extends LinearOpMode {

    // Robot Instance
    private final Robot robot = new Robot();

    private boolean flyWheelOn = false;
    private int RPS = 10;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("Status: Initialized. Ready to start.");
        telemetry.update();

        // Wait for the Start button to be pressed on the Driver Station.
        waitForStart();

        while (opModeIsActive()) {

            double drive   = -gamepad1.right_stick_y;
            double rotate  =  gamepad1.left_stick_x;

            double leftPower  = drive + rotate;
            double rightPower = drive - rotate;

            // Prevents Motors from Exceeding 100% Power
            double maxAbs = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxAbs > 1.0) {
                leftPower /= maxAbs;
                rightPower /= maxAbs;
            }

            robot.driveTrain.tankDrive(leftPower, rightPower);

            // Brake DriveTrain if No Joystick Input (1e-3 (0.001) Accounts for Stick Drift)
            if (Math.abs(drive) <= 1e-3 && Math.abs(rotate) <= 1e-3) {
                robot.driveTrain.brake();
            }

            // Roller Intake Control
            double intakeInput = gamepad1.right_trigger;
            double maxIntakeVelocity = 50; // Value in Rotations Per Second
            double targetIntakeVelocity = intakeInput * (maxIntakeVelocity * 360.0);
            double measuredIntakeRps = (robot.scoringMechanisms.flyWheel1.getVelocity(AngleUnit.DEGREES) / 360.0);

            robot.scoringMechanisms.rollerIntake.setVelocity(targetIntakeVelocity, AngleUnit.DEGREES);

            telemetry.addData("Roller Intake Trigger", intakeInput);
            telemetry.addData("Target Velocity (RPS)", targetIntakeVelocity / 360.0);
            telemetry.addData("Measured Velocity (RPS)", measuredIntakeRps);

            // Fly Wheel Control
            if (gamepad1.rightBumperWasPressed()) {
                flyWheelOn = !flyWheelOn;
            }

            if (gamepad1.dpadUpWasPressed() && (RPS < 50)) {
                RPS++;
            }

            if (gamepad1.dpadDownWasPressed() && (RPS > 0)) {
                RPS--;
            }

            double targetFlywheelRps = flyWheelOn ? RPS : 0.0;

            double measuredFlywheelRps = (robot.scoringMechanisms.flyWheel1.getVelocity(AngleUnit.DEGREES) / 360.0);
            robot.scoringMechanisms.flyWheel1.setVelocity((targetFlywheelRps * 360.0), AngleUnit.DEGREES);
            robot.scoringMechanisms.flyWheel2.setVelocity((targetFlywheelRps * 360.0), AngleUnit.DEGREES);

            telemetry.addData("Flywheel", flyWheelOn ? "(ON)" : "(OFF)");
            telemetry.addData("Target Velocity (RPS)", targetFlywheelRps);
            telemetry.addData("Measured Velocity (RPS)", measuredFlywheelRps);
            telemetry.update();

        }
    }
}
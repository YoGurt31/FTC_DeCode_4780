package Systems;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Title: Robot - Container Class for Decode Robot
 * Desc: Container Class for ALL Robot Subsystems
 * Includes Universal and General Functions
 *
 * @Author Gurtej Singh
 * @Version 1.0
 * adb connect 192.168.43.1:5555
 */

public class Robot {

    public class DriveTrain {
        // Hardware Devices
        public DcMotorEx frontLeft, frontRight, backLeft, backRight;
        public GoBildaPinpointDriver pinPoint;

        public void init(HardwareMap hardwareMap) {

            // Initialize DcMotors - Name in " " should match Driver Station Configuration
            frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
            frontRight = hardwareMap.get(DcMotorEx.class, "fR");
            backLeft = hardwareMap.get(DcMotorEx.class, "bL");
            backRight = hardwareMap.get(DcMotorEx.class, "bR");

            // Set Motor Directions - Positive Power should Drives Forward
            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            backLeft.setDirection(DcMotorEx.Direction.REVERSE);
            frontRight.setDirection(DcMotorEx.Direction.FORWARD);
            backRight.setDirection(DcMotorEx.Direction.FORWARD);

            // Brake when Power = 0 (Helps Negate Momentum)
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Stops Motors and Resets Encoders - Motors will NOT Run unless Encoder Mode is Defined
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            // Encoder Mode Definition - Run With or Without Encoders
            frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            // TODO: ENCODER DIRECTIONS
            pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint");
            pinPoint.setOffsets(-176, 68, DistanceUnit.MM);
            pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinPoint.resetPosAndIMU();
            pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }

        public void tankDrive(double Drive, double Rotate) {
            double leftPower = Drive - Rotate;
            double rightPower = Drive + Rotate;

            // Prevents Motors from Exceeding 100% Power
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));

            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            frontLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backLeft.setPower(leftPower);
            backRight.setPower(rightPower);
        }

        public void brake() {
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public void driveStraight(LinearOpMode opMode, double distanceInches, double maxPower) {
            pinPoint.resetPosAndIMU();
            pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

            double direction = Math.signum(distanceInches);
            double target = Math.abs(distanceInches);

            // TODO: Tune Values
            final double kP_drive = 0.05;
            final double kP_turn  = 0.01;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                Pose2D pose = pinPoint.getPosition();

                double x = pose.getX(DistanceUnit.INCH);
                double y = pose.getY(DistanceUnit.INCH);
                double headingDeg = pose.getHeading(AngleUnit.DEGREES);

                double travelled = Math.hypot(x, y);
                double remaining = target - travelled;

                if (remaining <= 0.5) break;

                double driveCmd = Range.clip(remaining * kP_drive * direction, -maxPower, maxPower);

                double headingError = normalizeAngle(0.0 - headingDeg);
                double rotateCmd = Range.clip(headingError * kP_turn, -0.3, 0.3);

                tankDrive(driveCmd, rotateCmd);
                opMode.idle();
            }

            tankDrive(0, 0);
        }

        public void turnToHeading(LinearOpMode opMode, double targetHeadingDeg, double maxTurnPower) {
            final double kP_turn = 0.01;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                double headingDeg = pinPoint.getHeading(AngleUnit.DEGREES);
                double error = normalizeAngle(targetHeadingDeg - headingDeg);

                if (Math.abs(error) <= 1.0) break;

                double rotateCmd = Range.clip(error * kP_turn, -maxTurnPower, maxTurnPower);

                tankDrive(0.0, rotateCmd);
                opMode.idle();
            }

            tankDrive(0, 0);
        }

        public Pose2D getPose() {
            pinPoint.update();
            return pinPoint.getPosition();
        }

        private double normalizeAngle(double angle) {
            while (angle >= 180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        }

    }

    public class ScoringMechanisms {
        // Hardware Devices
        public DcMotorEx rollerIntake, sorterIntake, flyWheel1, flyWheel2;
        public Servo leftRelease, rightRelease;

        public void init(HardwareMap hardwareMap) {

            rollerIntake = hardwareMap.get(DcMotorEx.class, "rI");
            // TODO: Tune Direction
            rollerIntake.setDirection(DcMotorEx.Direction.FORWARD);
            rollerIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rollerIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rollerIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sorterIntake = hardwareMap.get(DcMotorEx.class, "sI");
            // TODO: Tune Direction
            sorterIntake.setDirection(DcMotorEx.Direction.FORWARD);
            sorterIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sorterIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sorterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // TODO: Determine Which Motor Port ELC Encoder Is Connected Too

            flyWheel1 = hardwareMap.get(DcMotorEx.class, "fW1");
            // TODO: Tune Direction
            flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            flyWheel2 = hardwareMap.get(DcMotorEx.class, "fW2");
            // TODO: Tune Direction
            flyWheel2.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // TODO: Tune Directions
            leftRelease = hardwareMap.get(Servo.class, "lR");
            leftRelease.setPosition(1.0);
            leftRelease.setDirection(Servo.Direction.FORWARD);
            rightRelease = hardwareMap.get(Servo.class, "rR");
            rightRelease.setPosition(1.0);
            rightRelease.setDirection(Servo.Direction.REVERSE);
        }
    }

    public class Vision {
        public org.firstinspires.ftc.vision.VisionPortal visionPortal;
        public AprilTagProcessor aprilTag;
        public AprilTagDetection desiredTag = null;

        public void init(HardwareMap hardwareMap) {
            visionPortal = org.firstinspires.ftc.vision.VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam"), aprilTag);
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        }
    }

    // Created Instances of Subsystems
    public DriveTrain driveTrain = new DriveTrain();
    public ScoringMechanisms scoringMechanisms = new ScoringMechanisms();
    public Vision vision = new Vision();

    // Initialize Hardware
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
        scoringMechanisms.init(hwMap);
        vision.init(hwMap);
    }
}

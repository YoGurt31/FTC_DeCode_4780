package Systems;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Title: Robot - Container Class for Decode Robot
 * Desc: Container Class for ALL Robot Subsystems
 * Includes Universal and General Functions
 *
 * @Author Gurtej Singh
 * adb connect 192.168.43.1:5555
 */

public class Robot {

    public static class DriveTrain {
        // Hardware Devices
        public DcMotorEx frontLeft, frontRight, backLeft, backRight;
        public GoBildaPinpointDriver pinPoint;
        public Servo gearShiftLeft, gearShiftRight;

        public void init(HardwareMap hardwareMap) {

            // Initialize DcMotors - Name in " " should match Driver Station Configuration
            frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
            frontRight = hardwareMap.get(DcMotorEx.class, "fR");
            backLeft = hardwareMap.get(DcMotorEx.class, "bL");
            backRight = hardwareMap.get(DcMotorEx.class, "bR");

            // Set Motor Directions - Positive Power should Drives Forward
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            backLeft.setDirection(DcMotorEx.Direction.FORWARD);
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            backRight.setDirection(DcMotorEx.Direction.REVERSE);

            // Float when Power = 0 (Helps Conserve Battery)
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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

            // PinPoint Localizer
            pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinPoint.setOffsets(-176, -66, DistanceUnit.MM); // TODO: Tune Values
            pinPoint.resetPosAndIMU();
            pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

            // GearShift SetUp // TODO: Tune
            gearShiftLeft = hardwareMap.get(Servo.class, "gearShiftLeft");
            gearShiftLeft.setDirection(Servo.Direction.FORWARD);
            gearShiftLeft.setPosition(0.0);

            gearShiftRight = hardwareMap.get(Servo.class, "gearShiftRight");
            gearShiftRight.setDirection(Servo.Direction.REVERSE);
            gearShiftRight.setPosition(0.0);
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
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        public void driveForwardInches(LinearOpMode opMode, double inches, double power) {
            if (pinPoint == null || opMode == null) return;

            power = Range.clip(power, -1.0, 1.0);
            if (inches <= 0.0 || Math.abs(power) < 1e-3) {
                return;
            }

            pinPoint.update();
            Pose2D startPose = pinPoint.getPosition();
            double startX = startPose.getX(DistanceUnit.INCH);
            double startY = startPose.getY(DistanceUnit.INCH);
            double targetHeadingRad = startPose.getHeading(AngleUnit.RADIANS);

            final double kHeading = 0.75;
            final double maxRotate = 0.40;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                Pose2D currPose = pinPoint.getPosition();
                double currX = currPose.getX(DistanceUnit.INCH);
                double currY = currPose.getY(DistanceUnit.INCH);
                double currHeadingRad = currPose.getHeading(AngleUnit.RADIANS);

                double dx = currX - startX;
                double dy = currY - startY;
                double traveled = Math.hypot(dx, dy);

                if (traveled >= inches) {
                    break;
                }

                double headingError = AngleUnit.normalizeRadians(targetHeadingRad - currHeadingRad);
                double rotateCmd = Range.clip(headingError * kHeading, -maxRotate, maxRotate);

                tankDrive(power, rotateCmd);

                opMode.idle();
            }

            tankDrive(0.0, 0.0);
            brake();
        }

    }

    public static class ScoringMechanisms {
        // Hardware Devices
        public DcMotorEx rollerIntake, sorterIntake, flyWheel1, flyWheel2;
        public Servo leftRelease, rightRelease;

        public void init(HardwareMap hardwareMap) {

            rollerIntake = hardwareMap.get(DcMotorEx.class, "rI");
            rollerIntake.setDirection(DcMotorEx.Direction.FORWARD);
            rollerIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rollerIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rollerIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sorterIntake = hardwareMap.get(DcMotorEx.class, "sI");
            sorterIntake.setDirection(DcMotorEx.Direction.FORWARD);
            sorterIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sorterIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sorterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double P = 50.000;
            double I = 00.000;
            double D = 00.000;
            double F = 15.000;

            flyWheel1 = hardwareMap.get(DcMotorEx.class, "fW1");
            flyWheel1.setDirection(DcMotorEx.Direction.FORWARD);
            flyWheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flyWheel1.setVelocityPIDFCoefficients(P, I, D, F);

            flyWheel2 = hardwareMap.get(DcMotorEx.class, "fW2");
            flyWheel2.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flyWheel2.setVelocityPIDFCoefficients(P, I, D, F);

            leftRelease = hardwareMap.get(Servo.class, "lR");
            rightRelease = hardwareMap.get(Servo.class, "rR");
        }
    }

    public static class Vision {
        public Limelight3A limeLight;
        public int desiredPipeline = 0;
        public int RED = 1;
        public int BLUE = 2;
        public int motifTagId = -1;
        public String motifPattern = "UNKNOWN";

        public void init(HardwareMap hardwareMap) {
            limeLight = hardwareMap.get(Limelight3A.class, "limelight");
            limeLight.pipelineSwitch(0);
            limeLight.start();
        }

        public void setPipeline(int pipelineIndex) {
            desiredPipeline = pipelineIndex;
            if (limeLight != null) {
                try {
                    limeLight.pipelineSwitch(desiredPipeline);
                } catch (Exception ignored) {}
            }
        }

        public void maintainPipeline() {
            if (limeLight == null) return;
            try {
                int current = limeLight.getLatestResult().getPipelineIndex();
                if (current != desiredPipeline) {
                    limeLight.pipelineSwitch(desiredPipeline);
                }
            } catch (Exception ignored) {
            }
        }

        public void reboot() {
            if (limeLight == null) return;
            try {
                limeLight.stop();
            } catch (Exception ignored) {}

            try {
                limeLight.start();
                limeLight.pipelineSwitch(desiredPipeline);
            } catch (Exception ignored) {}
        }

        public void updateMotif() {
            if (motifTagId == 21 || motifTagId == 22 || motifTagId == 23) {
                return;
            }

            if (limeLight == null) {
                return;
            }

            LLResult result = limeLight.getLatestResult();
            if (result == null || !result.isValid()) {
                return;
            }

            java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                return;
            }

            int id = fiducials.get(0).getFiducialId();

            switch (id) {
                case 21:
                    motifTagId = 21;
                    motifPattern = "GPP";
                    break;
                case 22:
                    motifTagId = 22;
                    motifPattern = "PGP";
                    break;
                case 23:
                    motifTagId = 23;
                    motifPattern = "PPG";
                    break;
                default:
                    break;
            }
        }

        public boolean hasMotif() {
            return motifTagId == 21 || motifTagId == 22 || motifTagId == 23;
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
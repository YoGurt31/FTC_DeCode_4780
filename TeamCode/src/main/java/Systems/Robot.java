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
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    public static class DriveTrain {
        // Hardware Devices
        public DcMotorEx frontLeft, frontRight, backLeft, backRight;
        public GoBildaPinpointDriver pinPoint;
        public Servo gearShift1, gearShift2;

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

            // PinPoint Localizer
            pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinPoint.setOffsets(-176, -66, DistanceUnit.MM);
            pinPoint.resetPosAndIMU();
            pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

//            // GearShift SetUp
//            gearShift1 = hardwareMap.get(Servo.class, "gearShift1");
//            gearShift2 = hardwareMap.get(Servo.class, "gearShift2");
//            gearShift1.setDirection(Servo.Direction.FORWARD);
//            gearShift2.setDirection(Servo.Direction.FORWARD);
//            gearShift1.setPosition(0.0);
//            gearShift2.setPosition(0.0);
        }

        public void tankDrive(double Drive, double Rotate) {
            double leftPower = Drive + Rotate;
            double rightPower = Drive - Rotate;

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

        // Motion Functions
        public void driveDistance(LinearOpMode opMode, double distanceInches, double power) {
            if (opMode == null) return;
            if (pinPoint == null) return;

            double dir = Math.signum(distanceInches);
            double drivePower = Math.abs(power) * dir;

            if (Math.abs(distanceInches) < 1e-2 || Math.abs(power) < 1e-2) {
                tankDrive(0.0, 0.0);
                return;
            }

            pinPoint.update();
            Pose2D start = pinPoint.getPosition();
            double startX = start.getX(DistanceUnit.INCH);
            double startY = start.getY(DistanceUnit.INCH);
            double targetHeadingDeg = start.getHeading(AngleUnit.DEGREES);

            final double headingKp = 0.02;
            final double maxCorrection = 0.35;
            final double threshold = 0.5;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                Pose2D cur = pinPoint.getPosition();

                double x = cur.getX(DistanceUnit.INCH);
                double y = cur.getY(DistanceUnit.INCH);
                double headingDeg = cur.getHeading(AngleUnit.DEGREES);

                double dx = x - startX;
                double dy = y - startY;
                double traveled = Math.hypot(dx, dy);
                double remaining = Math.abs(distanceInches) - traveled;

                double headingErr = wrapDeg(targetHeadingDeg - headingDeg);
                double correction = Range.clip(headingErr * headingKp, -maxCorrection, maxCorrection);

                if (remaining <= threshold) break;

                tankDrive(drivePower, correction);
                opMode.idle();
            }

            tankDrive(0.0, 0.0);
        }

        public void turn(LinearOpMode opMode, String direction, double angleDeg) {
            if (opMode == null) return;
            if (pinPoint == null) return;

            if (Math.abs(angleDeg) < 1e-2) {
                tankDrive(0.0, 0.0);
                return;
            }

            double dir;
            if (direction == null) {
                dir = 1.0;
            } else {
                String d = direction.trim().toLowerCase();
                if (d.startsWith("l")) dir = 1.0;
                else if (d.startsWith("r")) dir = -1.0;
                else dir = 1.0;
            }

            pinPoint.update();
            double startHeadingDeg = pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
            double targetHeadingDeg = wrapDeg(startHeadingDeg + dir * angleDeg);

            final double turnKp = 0.05;
            final double minRotate = 0.10;
            final double maxRotate = 0.50;
            final double threshold = 1.0;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                double headingDeg = pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
                double err = wrapDeg(targetHeadingDeg - headingDeg);

                if (Math.abs(err) <= threshold) break;

                double rotate = Range.clip(err * turnKp, -maxRotate, maxRotate);

                if (rotate > 0 && Math.abs(rotate) < minRotate) rotate = minRotate;
                if (rotate < 0 && Math.abs(rotate) < minRotate) rotate = -minRotate;

                tankDrive(0.0, rotate);
                opMode.idle();
            }

            tankDrive(0.0, 0.0);
        }

        public void turnTo(LinearOpMode opMode, double headingDeg) {
            if (opMode == null) return;
            if (pinPoint == null) return;

            pinPoint.update();
            double targetHeadingDeg = wrapDeg(headingDeg);

            final double turnKp = 0.05;
            final double minRotate = 0.10;
            final double maxRotate = 0.50;
            final double thresholdDeg = 1.0;

            while (opMode.opModeIsActive()) {
                pinPoint.update();
                double currentHeadingDeg = pinPoint.getPosition().getHeading(AngleUnit.DEGREES);

                double errDeg = wrapDeg(targetHeadingDeg - currentHeadingDeg);

                if (Math.abs(errDeg) <= thresholdDeg) break;

                double rotate = Range.clip(errDeg * turnKp, -maxRotate, maxRotate);

                if (rotate > 0 && Math.abs(rotate) < minRotate) rotate = minRotate;
                if (rotate < 0 && Math.abs(rotate) < minRotate) rotate = -minRotate;

                tankDrive(0.0, rotate);
                opMode.idle();
            }

            tankDrive(0.0, 0.0);
        }
        
        private static double wrapDeg(double deg) {
            while (deg > 180.0) deg -= 360.0;
            while (deg <= -180.0) deg += 360.0;
            return deg;
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

            double F = 14.385;
            double P = 2 * F;
            double I = 00.000;
            double D = 00.000;

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

        // VARIABLES
        public double targetRPS = 0.0;
        public final double farTargetRPS = 54.5;
        public final double closeTargetRPS = 48.5;
        public final double TicksPerRev = 28.0;
        public final double artifactHoldRight = 0.5;
        public final double artifactHoldLeft = 0.0;
        public final double artifactReleaseRight = 1.0;
        public final double artifactReleaseLeft = 0.5;
        public long leftShotEndTime = 0;
        public long rightShotEndTime = 0;
        public long leftGateOpenUntil = 0;
        public long rightGateOpenUntil = 0;
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

        // VARIABLES
        public final double rotateGain = 0.0250;
        public final double maxRotate = 0.75;
        public final double tagAreaThreshold = 0.8;
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
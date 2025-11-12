package Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

        public void init(HardwareMap hardwareMap) {

            // Initialize DcMotors - Name in " " should match Driver Station Configuration
            frontLeft = hardwareMap.get(DcMotorEx.class, "fL");
            frontRight = hardwareMap.get(DcMotorEx.class, "fR");
            backLeft = hardwareMap.get(DcMotorEx.class, "bL");
            backRight = hardwareMap.get(DcMotorEx.class, "bR");

            // TODO
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

        }

        public void tankDrive(double leftPower, double rightPower) {
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

    }

    public class ScoringMechanisms {
        // Hardware Devices
        public DcMotorEx rollerIntake, sorterIntake, flyWheel1, flyWheel2;

        public void init(HardwareMap hardwareMap) {

            rollerIntake = hardwareMap.get(DcMotorEx.class, "rI");
            // TODO
            rollerIntake.setDirection(DcMotorEx.Direction.FORWARD);
            rollerIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rollerIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rollerIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sorterIntake = hardwareMap.get(DcMotorEx.class, "sI");
            // TODO
            sorterIntake.setDirection(DcMotorEx.Direction.FORWARD);
            sorterIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sorterIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sorterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // TODO: Determine Which Motor Port ELC Encoder Is Connected Too

            flyWheel1 = hardwareMap.get(DcMotorEx.class, "fW1");
            // TODO
            flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flyWheel2 = hardwareMap.get(DcMotorEx.class, "fW2");
            // TODO
            flyWheel2.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // Created Instances of Subsystems
    public DriveTrain driveTrain = new DriveTrain();
    public ScoringMechanisms scoringMechanisms = new ScoringMechanisms();

    // Initialize Hardware
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
        scoringMechanisms.init(hwMap);
    }
}

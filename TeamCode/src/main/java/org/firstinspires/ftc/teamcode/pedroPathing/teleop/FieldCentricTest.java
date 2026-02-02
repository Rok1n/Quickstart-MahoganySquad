package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Test", group = "Test")
public class FieldCentricTest extends LinearOpMode {

    // Define chassis motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    // Define IMU (Gyroscope), the core of spatial awareness
    private IMU imu;

    @Override
    public void runOpMode() {
        // 1. Hardware Mapping (names must match your Driver Station config)
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");

        // 2. Set Motor Directions (Based on your drive.java config)
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // Right side is usually FORWARD, change if wheels spin wrong way
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Zero Power Behavior (Brake when zero power, gives better control)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Initialize IMU (Gyro) - CRITICAL STEP!
        // Check how your Control Hub is mounted on the robot:
        // The settings below assume: Hub is laying flat (Logo UP), USB port facing forward (USB FORWARD)
        // If your Hub is mounted vertically, you MUST change these parameters!
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addLine("Initialization Complete!");
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Left Stick moves relative to the FIELD");
        telemetry.addLine("2. Right Stick rotates the robot");
        telemetry.addLine("3. Press [Options/Start] to reset forward direction");
        telemetry.update();

        waitForStart();//

        while (opModeIsActive()) {
            // === Core Logic ===

            // 1. Get Joystick Inputs
            double y = -gamepad1.left_stick_y; // Forward/Back (Stick is negative when up, so invert)
            double x = gamepad1.left_stick_x;  // Left/Right
            double rx = gamepad1.right_stick_x; // Rotation

            // 2. Reset Heading: If driver feels "Forward" is off, press this to reset
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // 3. Get current robot heading (in radians)
            double botHeading = imu.getRobotYawPitchRoll().yaw;

            // 4. Field Centric Math (Coordinate Rotation)
            // This is the magic that makes the robot "forget its head and follow the field"
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Compensate for strafing friction (Mecanum wheels are slower sideways, multiply by 1.1)
            rotX = rotX * 1.1;

            // 5. Calculate Motor Powers
            // Use the rotated rotX and rotY (field relative values)
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // 6. Set Motor Powers
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // === Telemetry ===
            telemetry.addData("Mode", "Field Centric");
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}
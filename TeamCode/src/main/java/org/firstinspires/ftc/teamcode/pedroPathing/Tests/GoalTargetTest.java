package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.VisionData;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;

@TeleOp(name = "Goal Target Test", group = "Test")
public class GoalTargetTest extends LinearOpMode {

    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private Limelight limelight;
    private GoalTargeter goalTargeter;

    private static final int BLUE_GOAL_TAG_ID = 20;

    @Override
    public void runOpMode() {
        // --- Hardware Map ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        // --- Motor Configuration ---
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Subsystems ---
        limelight = new Limelight();
        limelight.init(hardwareMap);
        limelight.switchPipeline(0); // AprilTag pipeline
        goalTargeter = new GoalTargeter(limelight);

        telemetry.addLine("Ready. Press A to lock on AprilTag 20.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            goalTargeter.update();

            double x = Math.pow(gamepad1.left_stick_x, 3);
            double y = -Math.pow(gamepad1.left_stick_y, 3);
            double rz = Math.pow(gamepad1.right_stick_x, 3);

            boolean isLocking = gamepad1.a;
            boolean tagDetected = goalTargeter.hasTarget() && goalTargeter.getVisionData().getTagID() == BLUE_GOAL_TAG_ID;

            if (isLocking) {
                // Ground the robot: zero out all translation (x and y) 
                // so it only rotates to face the tag.
                x = 0;
                y = 0;

                if (tagDetected) {
                    // Override rotation with steering correction to face tag
                    rz = -goalTargeter.getSteeringCorrection();
                } else {
                    // If no tag is seen, stop rotating as well
                    rz = 0;
                }
            }

            // Mecanum Drive Math
            double lbPower = (y + x) + rz;
            double lfPower = y - x + rz;
            double rbPower = (y - x) - rz;
            double rfPower = (y + x) - rz;

            // Normalize powers
            double max = Math.max(1.0, Math.max(Math.abs(lbPower), Math.max(Math.abs(lfPower), 
                       Math.max(Math.abs(rbPower), Math.abs(rfPower)))));
            
            leftBack.setPower(lbPower / max);
            leftFront.setPower(lfPower / max);
            rightBack.setPower(rbPower / max);
            rightFront.setPower(rfPower / max);

            // Telemetry
            telemetry.addData("Locking (A)", isLocking);
            telemetry.addData("Tag 20 Detected", tagDetected);
            telemetry.addData("TX", goalTargeter.getTx());
            telemetry.addData("Steer Corr", rz);
            telemetry.addData("Targeter Info", goalTargeter.getTelemetryString());
            telemetry.update();
        }
    }
}

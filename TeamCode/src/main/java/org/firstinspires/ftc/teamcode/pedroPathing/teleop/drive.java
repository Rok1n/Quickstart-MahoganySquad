package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.panels.Panels;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.VisionData;

@TeleOp(name = "drive")
public class drive extends LinearOpMode {

    // --- Hardware Declarations ---
    private DcMotor rightFront, rightBack, leftBack, leftFront;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelMotor;

    private DcMotor chamberSpinner;
    public CRServo artifactTransfer;
    
    private Servo LimeServo;
    
    // Classified as Servo because it is plugged into the Servo section of the Hub
    private Servo flywheelReadyLed;

    // --- Subsystems ---
    private Limelight limelight;
    private GoalTargeter goalTargeter;

    // --- Logic Variables ---
    private boolean intaking = false;
    private double intakeSpeed = 1;
    private boolean flywheeling = false;
    private boolean goalLockEnabled = false;

    // --- Flywheel Velocities ---
    // Update these values to your preference!
    private double HIGH_VELOCITY = 1350; // 1298
    private double LOW_VELOCITY = 1025;   // Placeholder for lower speed

    // --- Slow Mode Variables ---
    private boolean slowMode = false;
    private double speedMultiplier = 1.0;

    // --- Stepper Variables ---
    private double chamberTargetPos = 0;
    private double ticksPerStep = 475.06; //A
    private double shootPosticks = 100.0; //B
    private double superReverseTicks = 30.0; //Y

    // --- Button State Trackers ---
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastLT = false;
    private boolean lastRT = false;
    private int flywheelMode = 0; // 0: off, 1: high, 2: low
    private int shootState = 0;

    private GamepadEx operatorOp;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        // --- Hardware Map ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        

        flywheelReadyLed = hardwareMap.get(Servo.class, "flywheelLed");
        LimeServo = hardwareMap.get(Servo.class, "axonLime");


        // --- Vision Initialization ---
        limelight = new Limelight();
        limelight.init(hardwareMap);
        limelight.switchPipeline(0); // AprilTag = 0 pipeline
        goalTargeter = new GoalTargeter(limelight);


        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);


        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        PIDFCoefficients pidfNew = new PIDFCoefficients(20.3025, 0, 0, 20.7020); //f=9
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);


        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);

        // Set starting position to center
        LimeServo.setPosition(0.5);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();
            operatorOp.readButtons();
            goalTargeter.update();

            // =========================================================
            //                     DRIVE LOGIC
            // =========================================================

            // Slow Mode
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                slowMode = !slowMode;
                if (slowMode) {
                    speedMultiplier = 0.35;
                    gamepad1.rumble(200);
                    gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    speedMultiplier = 1.0;
                    gamepad1.setLedColor(255, 0, 0, 1000);
                }
            }

            double x = Math.pow(gamepad1.left_stick_x, 3);
            double y = -Math.pow(gamepad1.left_stick_y, 3);
            double rz = Math.pow(gamepad1.right_stick_x, 3);

            // Goal LockOn
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                goalLockEnabled = !goalLockEnabled;
                if (goalLockEnabled) {
                    gamepad1.rumble(300);
                }
            }

            if (goalLockEnabled) {
                // robot can't move
                x = 0;
                y = 0;
                // Lock rotation to AprilTag 20, 24
                if (goalTargeter.hasTarget()) {
                    int tagID = goalTargeter.getVisionData().getTagID();
                    if (tagID == 20 || tagID == 24) {
                        rz = -goalTargeter.getSteeringCorrection();
                    } else {
                        rz = 0;
                    }
                } else {
                    rz = 0;
                }
            }

            leftBack.setPower(((y - x) + rz) * speedMultiplier);
            leftFront.setPower((y + x + rz) * speedMultiplier);
            rightBack.setPower(((y + x) - rz) * speedMultiplier);
            rightFront.setPower(((y - x) - rz) * speedMultiplier);


            // =========================================================
            //                  LIMELIGHT SERVO LOGIC 
            // =========================================================
            
            if (operatorOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                LimeServo.setPosition(0.75); // ~45 degrees right. BLUE SIDE
            } else if (operatorOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                LimeServo.setPosition(0.25); // ~45 degrees left. RED SIDE
            } else if (operatorOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                LimeServo.setPosition(0.5);  // Center
            }

            // =========================================================
            //                  FLYWHEEL LOGIC (Toggle Triggers)
            // =========================================================

            boolean ltPressed = gamepad2.left_trigger > 0.3;
            boolean rtPressed = gamepad2.right_trigger > 0.3;


            if (ltPressed && !lastLT) {
                if (flywheelMode == 1) flywheelMode = 0;
                else flywheelMode = 1;
            }

            if (rtPressed && !lastRT) {
                if (flywheelMode == 2) flywheelMode = 0;
                else flywheelMode = 2;
            }
            
            lastLT = ltPressed;
            lastRT = rtPressed;

            if (shootState == 0) {
                if (flywheelMode == 1) {
                    flywheelMotor.setVelocity(HIGH_VELOCITY);
                    flywheeling = true;
                    gamepad2.setLedColor(255, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
                } else if (flywheelMode == 2) {
                    flywheelMotor.setVelocity(LOW_VELOCITY);
                    flywheeling = true;
                    gamepad2.setLedColor(0, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    flywheelMotor.setVelocity(0);
                    flywheeling = false;
                }
            }


            if (flywheeling) {
                double target = (flywheelMode == 1) ? HIGH_VELOCITY : LOW_VELOCITY;
                // Use Math.abs(flywheelMotor.getVelocity()) to handle potential sign issues
                // and increased tolerance to 80 to ensure it catches when "at speed"
                if (Math.abs(Math.abs(flywheelMotor.getVelocity()) - target) < 75) { //80
                    if ((System.currentTimeMillis() % 1400) < 700) {
                        flywheelReadyLed.setPosition(1.0); // LED ON
                    } else {
                        flywheelReadyLed.setPosition(0.0); // LED OFF
                    }
                } else {
                    flywheelReadyLed.setPosition(0.0); // LED OFF
                }
            } else {
                flywheelReadyLed.setPosition(0.0);
            }

            // =========================================================
            //                  OTHER MECHANISMS
            // =========================================================

            // Intake Logic
            if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                intaking = !intaking;
                intakeMotor.setPower(intaking ? intakeSpeed : 0);
                gamepad1.rumble(200);
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }

            // Artifact Transfer
            boolean triggerPressed = gamepad1.right_trigger > 0.1;
            boolean dpadRightPressed = gamepad1.dpad_right;

            if ((triggerPressed || dpadRightPressed) && shootState == 0) {
                artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                artifactTransfer.setPower(1);
                gamepad1.rumble(150);
            } else if (shootState == 0) {
                artifactTransfer.setPower(0);
            }

            // Chamber Logic
            if (gamepad2.a && !lastA) moveChamberStep();
            if (gamepad2.b && !lastB) { chamberTargetPos += shootPosticks; updateChamber(); }
            if (gamepad2.y && !lastY) { chamberTargetPos -= 100; updateChamber(); }
            if (gamepad2.x && !lastX) { chamberTargetPos += superReverseTicks; updateChamber(); }

            lastA = gamepad2.a; lastB = gamepad2.b; lastY = gamepad2.y; lastX = gamepad2.x;

            // =========================================================
            //                       TELEMETRY
            // =========================================================
            telemetry.addData("DRIVE MODE", slowMode ? "SLOW (25%)" : "FULL POWER");
            telemetry.addData("Goal Lock (LB)", goalLockEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Flywheel Velocity", flywheelMotor.getVelocity());
            if(flywheeling) {
                telemetry.addData("Target", flywheelMode == 1 ? "HIGH ("+HIGH_VELOCITY+")" : "LOW ("+LOW_VELOCITY+")");
            }
            telemetry.addData("Chamber Pos", chamberSpinner.getCurrentPosition());
            telemetry.addData("Intaking", intaking);
            
            if (goalLockEnabled) {
                telemetry.addData("Vision Status", goalTargeter.hasTarget() ? "LOCKED" : "SEARCHING");
                telemetry.addData("Tag ID", goalTargeter.hasTarget() ? goalTargeter.getVisionData().getTagID() : "None");
            }

            telemetry.update();
        }
    }

    private void moveChamberStep() {
        chamberTargetPos += ticksPerStep;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.72);
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    private void updateChamber() {
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.5);
        gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }
}

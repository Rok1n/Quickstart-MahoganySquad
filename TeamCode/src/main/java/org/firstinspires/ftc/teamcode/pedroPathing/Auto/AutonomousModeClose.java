package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.MotifDetector; //cool2
import org.firstinspires.ftc.teamcode.pedroPathing.vision.VisionData;

@Autonomous(name = "Limelight Close (Motif v1.1.3)", group = "Auto")
public class AutonomousModeClose extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED,
        BLUE,
    }

    private Alliance selectedAlliance = Alliance.BLUE; // Default

    // ===================== SUBSYSTEMS =====================
    private Follower follower;
    private Limelight limelight;
    private GoalTargeter goalTargeter;
    private MotifDetector motifDetector;

    private DcMotorEx flywheelMotor;
    private DcMotor chamberSpinner;
    private DcMotor intakeMotor;
    private CRServo artifactTransfer;
    private Servo LimeServo;

    // ===================== TIMING =====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // New timer for intake sequencing
    private ElapsedTime intakeSeqTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        SCAN_MOTIF,
        SHOOT_PRELOADS,
        NAV_TO_PRE_INTAKE, // Go to X=48
        INTAKE_DRIVE, // Drive to X=16/9 with intake ON
        PICKUP_BALLS, // Wait for intake
        NAV_TO_SHOOT,
        ALIGN_AND_SHOOT,
        LEAVE_MARK,
        DONE,
    }

    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    private static final double SCAN_TIMEOUT_SEC = 2.5;
    private static final double NAV_TIMEOUT_SEC = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 2.0;

    // UPDATED: Changed from Power to Velocity based on Drive file
    private static final double SHOOT_VELOCITY = 1025; //CHANGE

    private static final double CHAMBER_WAIT = 1.7; //1.9, 1.0, 1.4, 2.4
    private static final double ATM_PUSH_TIME_FIRST = 0.9; //2.3
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;

    // --- Chamber Stepper Variables ---
    private final double TICKS_PER_STEP = 475.06; // (A Button)
    private final double SHOOT_POS_TICKS = 100; // (B Button) = 100
    private final double BACK_TO_INTAKE_TICKS = 100; // (Y Button) = 30
    private double chamberTargetPos = 0;

    // Intake Sequencing Variables
    private int intakeSeqStage = 0;
    private static final double INTAKE_SPIN_DELAY = 0.200; // 500ms
    private static final double INTAKE_FIRST_DELAY = 0.700;

    // ===================== FIELD COORDINATES =====================

    // --- BLUE COORDINATES ---
    // --- CHANGE BACK IF NEEDED ---
    private final Pose BLUE_START = new Pose(20.968, 122.296, Math.toRadians(325)); //x=62.13 y=7.03
    private final Pose BLUE_SHOOT = new Pose(52.686, 96.116, Math.toRadians(318)); //x=56 y=17, HEADING = 297

    // Blue Pre-Intake (Start driving from here)
    private final Pose BLUE_INTAKE_GPP = new Pose(64, 43, Math.toRadians(-180)); //x=51
    private final Pose BLUE_INTAKE_PGP = new Pose(60, 67, Math.toRadians(-180)); //y=58 x 60
    private final Pose BLUE_INTAKE_PPG = new Pose(58, 90, Math.toRadians(-180)); //y=54 x 56

    // Blue Intake End (Stop driving here)
    private final Pose BLUE_INTAKE_GPP_END = new Pose(37, 43, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_PGP_END = new Pose(37, 67, Math.toRadians(-180)); //x25
    private final Pose BLUE_INTAKE_PPG_END = new Pose(42, 90, Math.toRadians(-180));

    // --- RED COORDINATES ---
    private final Pose RED_START = new Pose(122.672, 122.457, Math.toRadians(215));
    private final Pose RED_SHOOT = new Pose(85, 85, Math.toRadians(225));

    // Red Pre-Intake (Mirrored X=48 -> X=88, Mirrored Y)
    private final Pose RED_INTAKE_GPP = new Pose(80, 20, Math.toRadians(0)); //x 82
    private final Pose RED_INTAKE_PGP = new Pose(86, 42, Math.toRadians(0)); //x=80e
    private final Pose RED_INTAKE_PPG = new Pose(80, 64, Math.toRadians(0)); //x 82

    // Red Intake End (Mirrored X=16 -> X=128, X=9 -> X=135)
    private final Pose RED_INTAKE_GPP_END = new Pose(112, 20, Math.toRadians(0));
    private final Pose RED_INTAKE_PGP_END = new Pose(112, 42, Math.toRadians(0));
    private final Pose RED_INTAKE_PPG_END = new Pose(115, 64, Math.toRadians(0));

    private final Pose LEAVE_MARK_RED = new Pose(97, 73, Math.toRadians(225));
    private final Pose LEAVE_MARK_BLUE = new Pose(48.569, 71.869, Math.toRadians(318));

    // Active Points
    private Pose startPose;
    private Pose shootPose;
    private Pose preIntakePose;
    private Pose finalIntakePose;

    private PathChain currentPath;

    // --- VARIABLES ---
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private int ballsShot = 0;
    private int shootSubState = 0;
    private boolean inSecondShootingPhase = false;
    private String decisionReason = "Waiting";

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        // ===================== SELECTION LOOP =====================
        while (!isStarted() && !isStopRequested()) {
            // TOGGLE LOGIC
            if (gamepad1.left_bumper) {
                selectedAlliance = Alliance.RED;
            } else if (gamepad1.right_bumper) {
                selectedAlliance = Alliance.BLUE;
            }

            // Vision Updates
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            // Telemetry-driven Servo positioning during selection
            if (selectedAlliance == Alliance.BLUE) {
                LimeServo.setPosition(0.75); // Move to right to see obelisk on blue
            } else {
                LimeServo.setPosition(0.25); // Move to left to see obelisk on red
            }

            // Telemetry
            telemetry.addLine("=== LIMELIGHT CLOSE (ALLIANCE SELECTION) ===");
            telemetry.addData("Selected Alliance", selectedAlliance);
            telemetry.addLine("LB = RED | RB = BLUE");
            telemetry.addLine();
            telemetry.addData("Motif (Live)", motifDetector.getDetectedMotif());
            telemetry.update();
        }

        // ===================== SETUP BASED ON SELECTION =====================
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
        }

        follower.setStartingPose(startPose);
        buildAndFollowPath(startPose, shootPose);
        flywheelMotor.setVelocity(SHOOT_VELOCITY);

        runtime.reset();
        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        // ===================== RUN LOOP =====================
        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            switch (currentState) {
                case SCAN_MOTIF:
                    runScanMotif();
                    break;
                case SHOOT_PRELOADS:
                    runShootPreloads();
                    break;
                case NAV_TO_PRE_INTAKE:
                    runNavToPreIntake();
                    break;
                case INTAKE_DRIVE:
                    runIntakeDrive();
                    break;
                case PICKUP_BALLS:
                    runPickupBalls();
                    break;
                case NAV_TO_SHOOT:
                    runNavToShoot();
                    break;
                case ALIGN_AND_SHOOT:
                    runAlignAndShoot();
                    break;
                case LEAVE_MARK:
                    runLeaveMark();
                    break;
                case DONE:
                    stopAllMechanisms();
                    follower.breakFollowing();
                    break;
            }
            updateTelemetry();
        }
    }

    private void initHardware() {
        follower = Constants.createFollower(hardwareMap);
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        chamberSpinner = hardwareMap.get(DcMotor.class, "chamberSpinner");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        artifactTransfer = hardwareMap.get(CRServo.class, "ATM");
        LimeServo = hardwareMap.get(Servo.class, "axonLime");

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(20.3025, 0, 0, 20.7020); //f=11 p =10
        flywheelMotor.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            pidfNew
        );

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);

        limelight = new Limelight();
        limelight.init(hardwareMap);
        limelight.switchPipeline(0);
        goalTargeter = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();
        
        // Default to center on hardware init
        LimeServo.setPosition(0.5);
    }

    // ===================== LOGIC =====================

    private void runScanMotif() {
        // Continuously check for confident detection while driving
        if (
            detectedMotif == MotifDetector.Motif.UNKNOWN &&
            motifDetector.hasConfidentDetection()
        ) {
            detectedMotif = motifDetector.getDetectedMotif();
            decisionReason = "Confident (Drive)";
            RobotLog.d("AUTO", "Motif Locked (Confident): " + detectedMotif);
        }

        if (!follower.isBusy()) {
            if (detectedMotif == MotifDetector.Motif.UNKNOWN) {
                MotifDetector.Motif lastSeen = motifDetector.getDetectedMotif();
                if (lastSeen != MotifDetector.Motif.UNKNOWN) {
                    detectedMotif = lastSeen;
                    decisionReason = "Arrived (Weak)";
                } else {
                    detectedMotif = MotifDetector.Motif.GPP;
                    decisionReason = "Arrived (Default)";
                }
                RobotLog.d("AUTO", "Motif Locked (Final): " + detectedMotif);
            }
            
            // Once we reach the first shoot position, center the camera for shots
            LimeServo.setPosition(0.5);
            
            transitionTo(AutoState.SHOOT_PRELOADS);
        }
    }

    private void runShootPreloads() {
        // Step 1: Start at Shoot Pos 3 Reg turns (A)
        if (follower.isBusy()) return;
        runShootingLogic(false);
    }

    private void runNavToPreIntake() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.3) {
            intakeMotor.setPower(1.0);

            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);

            intakeSeqStage = 0;

            buildAndFollowPath(preIntakePose, finalIntakePose);
            transitionTo(AutoState.INTAKE_DRIVE);
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            // Failsafe path
            intakeMotor.setPower(1.0);

            // Apply intake pos logic here as well for safety
            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);

            intakeSeqStage = 0;

            buildAndFollowPath(preIntakePose, finalIntakePose);
            transitionTo(AutoState.INTAKE_DRIVE);
        }
    }

    private void runIntakeDrive() {
        // Step 3: "then spin 3 reg times (A)"
        updateIntakeIndexing();

        if (!follower.isBusy() && stateTimer.seconds() > 0.3) {
            transitionTo(AutoState.PICKUP_BALLS);
        } else if (stateTimer.seconds() > 3.0) {
            transitionTo(AutoState.PICKUP_BALLS);
        }
    }

    private void runPickupBalls() {
        // CONTINUE SEQUENCING (In case drive finished before 3 spins)
        updateIntakeIndexing();

        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            //intakeMotor.setPower(0);
            buildAndFollowPath(finalIntakePose, shootPose);
            flywheelMotor.setVelocity(SHOOT_VELOCITY); // Start flywheel while moving back
            transitionTo(AutoState.NAV_TO_SHOOT);
        }
    }

    // Helper method for the chamber sequence
    private void updateIntakeIndexing() {
        switch (intakeSeqStage) {
            case 0:
                // First ball spin
                if (intakeSeqTimer.seconds() >= INTAKE_FIRST_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 1;
                }
                break;
            case 1:
                // Wait for delay, then Second ball spin
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 2;
                }
                break;
            case 2:
                // Wait for delay, then Third ball spin
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 3;
                }
                break;
            case 3:
                // Wait for delay, then Third ball spin
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 4;
                }
                break;
            case 4:
                // Done
                break;
        }
    }

    private void runNavToShoot() {
        if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
            intakeMotor.setPower(0);
            startSecondShootingPhase();
        } else if (stateTimer.seconds() > NAV_TIMEOUT_SEC) {
            startSecondShootingPhase();
        }
    }

    private void startSecondShootingPhase() {
        // Step 4: "then adjust to shoot pos (B)" -> Add 100
        chamberTargetPos += SHOOT_POS_TICKS;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(1);

        inSecondShootingPhase = true;
        ballsShot = 0;
        shootSubState = 0;
        transitionTo(AutoState.ALIGN_AND_SHOOT);
    }

    private void runAlignAndShoot() {
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        // Step 5: "then spin 3 reg times (A)"
        runShootingLogic(true);
    }

    private void runShootingLogic(boolean isSecondPhase) {
        switch (shootSubState) {
            case 0:
                // Only spin the chamber if it's NOT the first ball of the phase.
                if (ballsShot > 0) {
                    moveChamberStep();
                }
                shootTimer.reset();
                shootSubState = 1;
                break;
            case 1:
                if (shootTimer.seconds() >= CHAMBER_WAIT) {
                    artifactTransfer.setDirection(
                        DcMotorSimple.Direction.FORWARD
                    );
                    artifactTransfer.setPower(1);
                    shootTimer.reset();
                    shootSubState = 2;
                }
                break;
            case 2:
                double pushTime = (ballsShot == 0 && !isSecondPhase)
                    ? ATM_PUSH_TIME_FIRST
                    : ATM_PUSH_TIME_NORMAL;
                if (shootTimer.seconds() >= pushTime) {
                    artifactTransfer.setPower(0);
                    shootSubState = 3;
                }
                break;
            case 3:
                ballsShot++;
                if (ballsShot >= 3) {
                    flywheelMotor.setVelocity(0);
                    if (!isSecondPhase) {
                        setTargetForMotif();
                        transitionTo(AutoState.NAV_TO_PRE_INTAKE);
                    } else {
                        // All balls shot in second phase, go to LEAVE_MARK
                        Pose leavePose = (selectedAlliance == Alliance.BLUE) ? LEAVE_MARK_BLUE : LEAVE_MARK_RED;
                        buildAndFollowPath(shootPose, leavePose);
                        transitionTo(AutoState.LEAVE_MARK);
                    }
                } else {
                    shootSubState = 0;
                }
                break;
        }
    }

    private void runLeaveMark() {
        if (!follower.isBusy()) {
            transitionTo(AutoState.DONE);
        }
    }

    private void setTargetForMotif() {
        if (selectedAlliance == Alliance.BLUE) {
            switch (detectedMotif) {
                case GPP:
                    preIntakePose = BLUE_INTAKE_GPP;
                    finalIntakePose = BLUE_INTAKE_GPP_END;
                    break;
                case PGP:
                    preIntakePose = BLUE_INTAKE_PGP;
                    finalIntakePose = BLUE_INTAKE_PGP_END;
                    break;
                case PPG:
                    preIntakePose = BLUE_INTAKE_PPG;
                    finalIntakePose = BLUE_INTAKE_PPG_END;
                    break;
                default:
                    preIntakePose = BLUE_INTAKE_GPP;
                    finalIntakePose = BLUE_INTAKE_GPP_END;
                    break;
            }
        } else {
            // RED LOGIC
            switch (detectedMotif) {
                case GPP:
                    preIntakePose = RED_INTAKE_GPP;
                    finalIntakePose = RED_INTAKE_GPP_END;
                    break;
                case PGP:
                    preIntakePose = RED_INTAKE_PGP;
                    finalIntakePose = RED_INTAKE_PGP_END;
                    break;
                case PPG:
                    preIntakePose = RED_INTAKE_PPG;
                    finalIntakePose = RED_INTAKE_PPG_END;
                    break;
                default:
                    preIntakePose = RED_INTAKE_GPP;
                    finalIntakePose = RED_INTAKE_GPP_END;
                    break;
            }
        }

        RobotLog.d(
            "AUTO",
            "Targets: Pre=" +
                preIntakePose.toString() +
                " | Final=" +
                finalIntakePose.toString()
        );

        // Build first leg: Shoot -> Pre-Intake
        buildAndFollowPath(shootPose, preIntakePose);
    }

    private void buildAndFollowPath(Pose start, Pose end) {
        currentPath = follower
            .pathBuilder()
            .addPath(new BezierLine(start, end))
            .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
            .build();
        follower.followPath(currentPath);
    }

    private void transitionTo(AutoState newState) {
        currentState = newState;
        stateTimer.reset();
    }

    private void stopAllMechanisms() {
        flywheelMotor.setVelocity(0);
        intakeMotor.setPower(0);
        artifactTransfer.setPower(0);
    }

    private void moveChamberStep() {
        chamberTargetPos += TICKS_PER_STEP;
        chamberSpinner.setTargetPosition((int) chamberTargetPos);
        chamberSpinner.setPower(1); //0.6
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Motif", detectedMotif);
        // Added Velocity check
        telemetry.addData("Flywheel Vel", flywheelMotor.getVelocity());
        if (finalIntakePose != null) {
            telemetry.addData("Target Final X", "%.1f", finalIntakePose.getX());
        }
        telemetry.update();
    }
}

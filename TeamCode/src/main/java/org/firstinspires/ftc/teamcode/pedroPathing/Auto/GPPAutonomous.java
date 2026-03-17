package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "GPP Autonomous Far", group = "Auto")
public class GPPAutonomous extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED, BLUE
    }
    private Alliance selectedAlliance = Alliance.BLUE; // Default

    // ===================== SUBSYSTEMS =====================
    private Follower follower;
    private GoalTargeter goalTargeter;

    private DcMotorEx flywheelMotor;
    private DcMotor chamberSpinner;
    private DcMotor intakeMotor;
    private CRServo artifactTransfer;

    // ===================== TIMING =====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime intakeSeqTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        SHOOT_PRELOADS,
        NAV_TO_PRE_INTAKE,
        INTAKE_DRIVE,
        PICKUP_BALLS,
        NAV_TO_SHOOT,
        ALIGN_AND_SHOOT,
        LEAVE_MARK,
        DONE
    }
    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    private static final double NAV_TIMEOUT_SEC = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 2.0;

    private static final double SHOOT_VELOCITY = 1280;
    private static final double CHAMBER_WAIT = 1.2; //change if needed

    private static final double FIRST_WAIT = 2.3;
    private static final double ATM_PUSH_TIME_FIRST = 1.8;
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;

    // Chamber Stepper Variables
    private final double TICKS_PER_STEP = 475.06;
    private final double SHOOT_POS_TICKS = 100;
    private final double BACK_TO_INTAKE_TICKS = 100;
    private double chamberTargetPos = 0;

    // Intake Sequencing Variables
    private int intakeSeqStage = 0;
    private static final double INTAKE_SPIN_DELAY = 0.400;

    // ===================== FIELD COORDINATES (GPP ONLY) =====================

    // --- BLUE COORDINATES ---
    private final Pose BLUE_START = new Pose(57, 8.5, Math.toRadians(270));
    private final Pose BLUE_SHOOT = new Pose(56, 15, Math.toRadians(293));
    private final Pose BLUE_INTAKE_GPP = new Pose(56, 34, Math.toRadians(-180));
    private final Pose BLUE_INTAKE_GPP_END = new Pose(25, 34, Math.toRadians(-180));

    // --- RED COORDINATES ---
    private final Pose RED_START = new Pose(87, 8.5, Math.toRadians(270));
    private final Pose RED_SHOOT = new Pose(88, 19, Math.toRadians(245)); //250
    private final Pose RED_INTAKE_GPP = new Pose(90, 19.5, Math.toRadians(0));
    private final Pose RED_INTAKE_GPP_END = new Pose(132, 19.5, Math.toRadians(0));

    private final Pose LEAVE_MARK_RED = new Pose(107.977, 13.269, Math.toRadians(270));
    private final Pose LEAVE_MARK_BLUE = new Pose(56, 40, Math.toRadians(270));

    // Active Points
    private Pose startPose;
    private Pose shootPose;
    private Pose preIntakePose;
    private Pose finalIntakePose;

    private PathChain currentPath;

    // --- VARIABLES ---
    private int ballsShot = 0;
    private int shootSubState = 0;
    private boolean inSecondShootingPhase = false;

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

            // Telemetry
            telemetry.addLine("=== GPP AUTONOMOUS (ALLIANCE SELECTION) ===");
            telemetry.addData("Selected Alliance", selectedAlliance);
            telemetry.addLine("LB = RED | RB = BLUE");
            telemetry.addLine();
            telemetry.addLine("This auto goes to GPP spike mark only.");
            telemetry.update();
        }

        // ===================== SETUP BASED ON SELECTION =====================
        if (selectedAlliance == Alliance.BLUE) {
            startPose = BLUE_START;
            shootPose = BLUE_SHOOT;
            preIntakePose = BLUE_INTAKE_GPP;
            finalIntakePose = BLUE_INTAKE_GPP_END;
        } else {
            startPose = RED_START;
            shootPose = RED_SHOOT;
            preIntakePose = RED_INTAKE_GPP;
            finalIntakePose = RED_INTAKE_GPP_END;
        }

        follower.setStartingPose(startPose);
        runtime.reset();

        // Go directly to shooting preloads (no motif scanning)
        buildAndFollowPath(startPose, shootPose);
        transitionTo(AutoState.SHOOT_PRELOADS);
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        stateTimer.reset();

        // ===================== RUN LOOP =====================
        while (opModeIsActive()) {
            follower.update();

            // Update GoalTargeter if available
            if (goalTargeter != null) {
                goalTargeter.update();
            }

            switch (currentState) {
                case SHOOT_PRELOADS:    runShootPreloads(); break;
                case NAV_TO_PRE_INTAKE: runNavToPreIntake(); break;
                case INTAKE_DRIVE:      runIntakeDrive(); break;
                case PICKUP_BALLS:      runPickupBalls(); break;
                case NAV_TO_SHOOT:      runNavToShoot(); break;
                case ALIGN_AND_SHOOT:   runAlignAndShoot(); break;
                case LEAVE_MARK:        runLeaveMark(); break;
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

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(20.3025, 0, 0, 20.7020);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        chamberSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        chamberSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chamberSpinner.setTargetPosition(0);
        chamberSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chamberSpinner.setPower(0.63);


        goalTargeter = null;
    }

    // ===================== LOGIC =====================

    private void runShootPreloads() {

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
            intakeMotor.setPower(1.0);

            chamberTargetPos -= BACK_TO_INTAKE_TICKS;
            chamberSpinner.setTargetPosition((int) chamberTargetPos);
            chamberSpinner.setPower(1);

            intakeSeqStage = 0;

            buildAndFollowPath(preIntakePose, finalIntakePose);
            transitionTo(AutoState.INTAKE_DRIVE);
        }
    }

    private void runIntakeDrive() {
        updateIntakeIndexing();

        if (!follower.isBusy() && stateTimer.seconds() > 0.3) {
            transitionTo(AutoState.PICKUP_BALLS);
        } else if (stateTimer.seconds() > 3.0) {
            transitionTo(AutoState.PICKUP_BALLS);
        }
    }

    private void runPickupBalls() {
        updateIntakeIndexing();

        if (stateTimer.seconds() > PICKUP_TIMEOUT_SEC) {
            buildAndFollowPath(finalIntakePose, shootPose);
            flywheelMotor.setVelocity(SHOOT_VELOCITY);
            transitionTo(AutoState.NAV_TO_SHOOT);
        }
    }

    private void updateIntakeIndexing() {
        switch (intakeSeqStage) {
            case 0:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 1;
                }
                break;
            case 1:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 2;
                }
                break;
            case 2:
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
                    moveChamberStep();
                    intakeSeqTimer.reset();
                    intakeSeqStage = 3;
                }
                break;

            case 3:
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
                    artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                    artifactTransfer.setPower(1); //cool
                    shootTimer.reset();
                    shootSubState = 2;
                }
                break;
            case 2:
                double pushTime = (ballsShot == 0 && !isSecondPhase) ? ATM_PUSH_TIME_FIRST : ATM_PUSH_TIME_NORMAL;
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
                        // GPP targets are already set in init, go to pre-intake
                        buildAndFollowPath(shootPose, preIntakePose);
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

    private void buildAndFollowPath(Pose start, Pose end) {
        currentPath = follower.pathBuilder()
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
        chamberSpinner.setPower(1);
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Flywheel Vel", flywheelMotor.getVelocity());
        telemetry.addData("Balls Shot", ballsShot);
        if (finalIntakePose != null) {
            telemetry.addData("Target Final X", "%.1f", finalIntakePose.getX());
        }
        telemetry.update();
    }
}

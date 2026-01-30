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
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.MotifDetector;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "6-Ball Auto Far (Motif v5.5.1)", group = "Auto")
public class AutonomousMode extends LinearOpMode {

    // ===================== ALLIANCE SELECTION =====================
    private enum Alliance {
        RED, BLUE
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

    // ===================== TIMING =====================
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();

    // New timer for intake sequencing
    private ElapsedTime intakeSeqTimer = new ElapsedTime();

    // ===================== STATE MACHINE =====================
    private enum AutoState {
        INIT,
        SCAN_MOTIF,
        SHOOT_PRELOADS,
        NAV_TO_PRE_INTAKE, // Go to X=48
        INTAKE_DRIVE,      // Drive to X=16/9 with intake ON
        PICKUP_BALLS,      // Wait for intake
        NAV_TO_SHOOT,
        ALIGN_AND_SHOOT,
        DONE
    }
    private AutoState currentState = AutoState.INIT;

    // ===================== CONFIGURATION =====================
    private static final double SCAN_TIMEOUT_SEC = 2.5;
    private static final double NAV_TIMEOUT_SEC = 5.0;
    private static final double PICKUP_TIMEOUT_SEC = 2.0;

    // UPDATED: Changed from Power to Velocity based on Drive file
    private static final double SHOOT_VELOCITY = 1200; //1200

    private static final double CHAMBER_WAIT = 2.5; //1.9, 1.0, 1.4, 2.4
    private static final double ATM_PUSH_TIME_FIRST = 2.0; //2.3
    private static final double ATM_PUSH_TIME_NORMAL = 0.9;

    // --- Chamber Stepper Variables ---
    private final double TICKS_PER_STEP = 475.06; // (A Button)
    private final double SHOOT_POS_TICKS = 100; // (B Button) = 100
    private final double BACK_TO_INTAKE_TICKS = 100; // (Y Button) = 30
    private double chamberTargetPos = 0;

    // Intake Sequencing Variables
    private int intakeSeqStage = 0;
    private static final double INTAKE_SPIN_DELAY = 0.200; // 500ms

    // ===================== FIELD COORDINATES =====================

    // --- BLUE COORDINATES ---
    // --- CHANGE BACK IF NEEDED ---
    private final Pose BLUE_START = new Pose(57, 8.5, Math.toRadians(270)); //x=62.13 y=7.03
    private final Pose BLUE_SHOOT = new Pose(56, 15, Math.toRadians(299)); //x=56 y=17, HEADING = 297

    // Blue Pre-Intake (Start driving from here)
    private final Pose BLUE_INTAKE_GPP = new Pose(56, 34, Math.toRadians(-180)); //x=56 y=34
    private final Pose BLUE_INTAKE_PGP = new Pose(56, 58, Math.toRadians(-180)); //y=43
    private final Pose BLUE_INTAKE_PPG = new Pose(56, 82, Math.toRadians(-180)); //y=67

    // Blue Intake End (Stop driving here)
    private final Pose BLUE_INTAKE_GPP_END = new Pose(29, 34, Math.toRadians(-180)); //x35
    private final Pose BLUE_INTAKE_PGP_END = new Pose(29, 58, Math.toRadians(-180)); //x35
    private final Pose BLUE_INTAKE_PPG_END = new Pose(35, 82, Math.toRadians(-180));

    // --- RED COORDINATES ---
    private final Pose RED_START = new Pose(87, 8.5, Math.toRadians(270));
    private final Pose RED_SHOOT = new Pose(88, 19, Math.toRadians(250));

    // Red Pre-Intake (Mirrored X=48 -> X=88, Mirrored Y)
    private final Pose RED_INTAKE_GPP = new Pose(88, 36, Math.toRadians(0));
    private final Pose RED_INTAKE_PGP = new Pose(88, 60, Math.toRadians(0));
    private final Pose RED_INTAKE_PPG = new Pose(88, 69, Math.toRadians(0));

    // Red Intake End (Mirrored X=16 -> X=128, X=9 -> X=135)
    private final Pose RED_INTAKE_GPP_END = new Pose(109, 36, Math.toRadians(0));
    private final Pose RED_INTAKE_PGP_END = new Pose(109, 60, Math.toRadians(0));
    private final Pose RED_INTAKE_PPG_END = new Pose(115, 69, Math.toRadians(0));

    // Active Points
    private Pose startPose;
    private Pose shootPose;
    private Pose preIntakePose;
    private Pose finalIntakePose;

    private PathChain currentPath;

    // --- VARIABLES ---
    private MotifDetector.Motif detectedMotif = MotifDetector.Motif.UNKNOWN;
    private ShootingController shootingController;
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

            // Telemetry
            telemetry.addLine("=== 6-BALL AUTO (ALLIANCE SELECTION) ===");
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
        runtime.reset();
        currentState = AutoState.SCAN_MOTIF;
        stateTimer.reset();

        // ===================== RUN LOOP =====================
        while (opModeIsActive()) {
            follower.update();
            goalTargeter.update();
            motifDetector.update(goalTargeter.getVisionData());

            switch (currentState) {
                case SCAN_MOTIF:        runScanMotif(); break;
                case SHOOT_PRELOADS:    runShootPreloads(); break;
                case NAV_TO_PRE_INTAKE: runNavToPreIntake(); break;
                case INTAKE_DRIVE:      runIntakeDrive(); break;
                case PICKUP_BALLS:      runPickupBalls(); break;
                case NAV_TO_SHOOT:      runNavToShoot(); break;
                case ALIGN_AND_SHOOT:   runAlignAndShoot(); break;
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


        PIDFCoefficients pidfNew = new PIDFCoefficients(10, 0, 0, 10); //f=11
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

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
        
        // 初始化射击控制器
        shootingController = new ShootingController();
    }

    // ===================== LOGIC =====================

    private void runScanMotif() {
        if (motifDetector.hasConfidentDetection()) {
            detectedMotif = motifDetector.getDetectedMotif();
            decisionReason = "Confident";
            lockInMotifAndGo();
            return;
        }

        if (stateTimer.seconds() > SCAN_TIMEOUT_SEC) {
            MotifDetector.Motif lastSeen = motifDetector.getDetectedMotif();
            if (lastSeen != MotifDetector.Motif.UNKNOWN) {
                detectedMotif = lastSeen;
                decisionReason = "Timeout (Weak)";
            } else {
                detectedMotif = MotifDetector.Motif.GPP;
                decisionReason = "Timeout (Default)";
            }
            lockInMotifAndGo();
        }
    }

    private void lockInMotifAndGo() {
        RobotLog.d("AUTO", "Motif Locked: " + detectedMotif);
        buildAndFollowPath(startPose, shootPose);
        transitionTo(AutoState.SHOOT_PRELOADS);
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        
        // 启动第一阶段射击
        shootingController.startShooting(false);
    }

    private void runShootPreloads() {
        if (follower.isBusy()) return;
        
        // 使用射击控制器
        if (shootingController.update()) {
            // 第一阶段射击完成,转入收集球阶段
            flywheelMotor.setVelocity(0);
            setTargetForMotif();
            transitionTo(AutoState.NAV_TO_PRE_INTAKE);
        }
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
                if (intakeSeqTimer.seconds() >= INTAKE_SPIN_DELAY) {
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

        // 启动第二阶段射击
        shootingController.startShooting(true);
        transitionTo(AutoState.ALIGN_AND_SHOOT);
    }

    private void runAlignAndShoot() {
        flywheelMotor.setVelocity(SHOOT_VELOCITY);
        
        // 使用射击控制器
        if (shootingController.update()) {
            // 第二阶段射击完成,自动程序结束
            flywheelMotor.setVelocity(0);
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

        RobotLog.d("AUTO", "Targets: Pre=" + preIntakePose.toString() + " | Final=" + finalIntakePose.toString());

        // Build first leg: Shoot -> Pre-Intake
        buildAndFollowPath(shootPose, preIntakePose);
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
        chamberSpinner.setPower(1); //0.6
    }

    // ===================== SHOOTING CONTROLLER =====================
    
    /**
     * 射击控制器 - 封装完整的射击序列逻辑
     * 
     * 射击流程:
     * 1. IDLE -> ROTATING_CHAMBER: 旋转腔室到下一个球
     * 2. ROTATING_CHAMBER -> WAITING_STABILIZATION: 等待腔室和飞轮稳定
     * 3. WAITING_STABILIZATION -> PUSHING_BALL: 推球进入发射器
     * 4. PUSHING_BALL -> BALL_SHOT: 球已发射,记录计数
     * 5. BALL_SHOT -> ROTATING_CHAMBER (如果还有球) 或 IDLE (完成)
     */
    private class ShootingController {
        /**
         * 射击状态枚举
         */
        private enum ShootState {
            IDLE,                    // 空闲状态
            ROTATING_CHAMBER,        // 旋转腔室
            WAITING_STABILIZATION,   // 等待稳定
            PUSHING_BALL,            // 推球
            BALL_SHOT                // 球已发射
        }
        
        // 状态变量
        private ShootState state = ShootState.IDLE;
        private ElapsedTime stateTimer = new ElapsedTime();
        private int ballsShot = 0;
        private boolean isSecondPhase = false;
        
        /**
         * 开始射击序列
         * 
         * @param secondPhase true 表示第二阶段射击(收集的球), false 表示第一阶段(预装球)
         */
        public void startShooting(boolean secondPhase) {
            this.isSecondPhase = secondPhase;
            this.ballsShot = 0;
            transitionToState(ShootState.ROTATING_CHAMBER);
            RobotLog.d("SHOOT", "Starting shooting sequence, phase=" + (secondPhase ? "2" : "1"));
        }
        
        /**
         * 更新射击状态机
         * 
         * @return true 如果射击序列完成(3个球已发射), false 如果仍在进行中
         */
        public boolean update() {
            switch (state) {
                case IDLE:
                    return false;
                    
                case ROTATING_CHAMBER:
                    return handleRotatingChamber();
                    
                case WAITING_STABILIZATION:
                    return handleWaitingStabilization();
                    
                case PUSHING_BALL:
                    return handlePushingBall();
                    
                case BALL_SHOT:
                    return handleBallShot();
                    
                default:
                    return false;
            }
        }
        
        /**
         * 处理旋转腔室状态
         */
        private boolean handleRotatingChamber() {
            moveChamberStep();
            transitionToState(ShootState.WAITING_STABILIZATION);
            return false;
        }
        
        /**
         * 处理等待稳定状态
         */
        private boolean handleWaitingStabilization() {
            if (stateTimer.seconds() >= CHAMBER_WAIT) {
                // 启动传送带推球
                artifactTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
                artifactTransfer.setPower(1);
                transitionToState(ShootState.PUSHING_BALL);
            }
            return false;
        }
        
        /**
         * 处理推球状态
         */
        private boolean handlePushingBall() {
            // 第一阶段的第一个球需要更长的推送时间
            double pushTime = (ballsShot == 0 && !isSecondPhase) 
                ? ATM_PUSH_TIME_FIRST 
                : ATM_PUSH_TIME_NORMAL;
                
            if (stateTimer.seconds() >= pushTime) {
                // 停止传送带
                artifactTransfer.setPower(0);
                transitionToState(ShootState.BALL_SHOT);
            }
            return false;
        }
        
        /**
         * 处理球已发射状态
         */
        private boolean handleBallShot() {
            ballsShot++;
            RobotLog.d("SHOOT", "Ball " + ballsShot + " shot (phase " + (isSecondPhase ? "2" : "1") + ")");
            
            if (ballsShot >= 3) {
                // 射击序列完成
                RobotLog.d("SHOOT", "Shooting sequence complete");
                transitionToState(ShootState.IDLE);
                return true;
            } else {
                // 继续射击下一个球
                transitionToState(ShootState.ROTATING_CHAMBER);
                return false;
            }
        }
        
        /**
         * 状态转换辅助方法
         */
        private void transitionToState(ShootState newState) {
            if (state != newState) {
                RobotLog.d("SHOOT", "State: " + state + " -> " + newState);
            }
            this.state = newState;
            this.stateTimer.reset();
        }
        
        /**
         * 获取已发射的球数
         */
        public int getBallsShot() {
            return ballsShot;
        }
        
        /**
         * 获取当前状态(用于调试)
         */
        public ShootState getState() {
            return state;
        }
        
        /**
         * 重置控制器
         */
        public void reset() {
            state = ShootState.IDLE;
            ballsShot = 0;
            isSecondPhase = false;
            stateTimer.reset();
            RobotLog.d("SHOOT", "Controller reset");
        }
    }


    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Motif", detectedMotif);
        telemetry.addData("Flywheel Vel", flywheelMotor.getVelocity());
        
        // 添加射击状态信息
        if (currentState == AutoState.SHOOT_PRELOADS || 
            currentState == AutoState.ALIGN_AND_SHOOT) {
            telemetry.addData("Shoot State", shootingController.getState());
            telemetry.addData("Balls Shot", shootingController.getBallsShot() + " / 3");
        }
        
        if (finalIntakePose != null) {
            telemetry.addData("Target Final X", "%.1f", finalIntakePose.getX());
        }
        telemetry.update();
    }
}

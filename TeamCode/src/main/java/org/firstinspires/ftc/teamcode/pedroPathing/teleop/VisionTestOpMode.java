git merge upstream/mainpackage org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.GoalTargeter;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.MotifDetector;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.VisionData;

/**
 * Test OpMode for the vision pipeline.
 * Demonstrates GoalTargeter auto-alignment, MotifDetector pattern
 * recognition, and color artifact detection.
 *
 * Controls:
 * - Left Stick: Manual driving (when not auto-aligning)
 * - Right Stick X: Rotation
 * - A: Toggle auto-alignment mode
 * - D-Pad Up: Switch to AprilTag pipeline
 * - D-Pad Down: Switch to GREEN color pipeline
 * - D-Pad Left: Switch to PURPLE color pipeline
 */
@TeleOp(name = "Vision Test", group = "Test")
public class VisionTestOpMode extends OpMode {

    private MecanumDrive drive;
    private Limelight limelight;
    private GoalTargeter goalTargeter;
    private MotifDetector motifDetector;

    private boolean autoAlignEnabled = false;
    private boolean lastAButton = false;

    @Override
    public void init() {
        // Initialize subsystems
        drive = new MecanumDrive();
        limelight = new Limelight();

        drive.init(hardwareMap);
        limelight.init(hardwareMap);

        // Initialize vision processors
        goalTargeter = new GoalTargeter(limelight);
        motifDetector = new MotifDetector();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Press A to toggle auto-align");
        telemetry.addData("", "D-Pad to switch pipelines");
    }

    @Override
    public void loop() {
        // Update vision systems
        goalTargeter.update();

        // Update motif detector with latest vision data
        VisionData visionData = goalTargeter.getVisionData();
        motifDetector.update(visionData);

        // Toggle auto-align with A button (rising edge)
        boolean aButton = gamepad1.a;
        if (aButton && !lastAButton) {
            autoAlignEnabled = !autoAlignEnabled;
        }
        lastAButton = aButton;

        // Pipeline switching
        if (gamepad1.dpad_up) {
            limelight.switchPipeline(Limelight.PIPELINE_APRILTAG);
        } else if (gamepad1.dpad_down) {
            limelight.switchPipeline(Limelight.PIPELINE_GREEN);
        } else if (gamepad1.dpad_left) {
            limelight.switchPipeline(Limelight.PIPELINE_PURPLE);
        }

        // Driving logic
        double forward, strafe, rotate;

        if (autoAlignEnabled && goalTargeter.hasTarget()) {
            // Auto-align mode: use vision corrections
            forward = goalTargeter.getDriveCorrection();
            strafe = 0;
            rotate = goalTargeter.getSteeringCorrection();
        } else {
            // Manual mode: use gamepad
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
        }

        drive.driveFieldRelative(forward, strafe, rotate);

        // Telemetry
        telemetry.addData("Mode", autoAlignEnabled ? "AUTO-ALIGN" : "MANUAL");

        // Active Pipeline
        String pipelineName;
        switch (limelight.getCurrentPipeline()) {
            case Limelight.PIPELINE_APRILTAG:
                pipelineName = "AprilTag";
                break;
            case Limelight.PIPELINE_GREEN:
                pipelineName = "GREEN Color";
                break;
            case Limelight.PIPELINE_PURPLE:
                pipelineName = "PURPLE Color";
                break;
            default:
                pipelineName = "Pipeline " + limelight.getCurrentPipeline();
        }
        telemetry.addData("Pipeline", pipelineName);
        telemetry.addLine();

        // GoalTargeter data
        telemetry.addData("Target", goalTargeter.hasTarget() ? "DETECTED" : "---");
        if (goalTargeter.hasTarget()) {
            telemetry.addData("  TX (horiz)", "%.1f°", goalTargeter.getTx());
            telemetry.addData("  TY (vert)", "%.1f°", goalTargeter.getTy());
            telemetry.addData("  TA (area)", "%.2f%%", goalTargeter.getTa());
            telemetry.addData("  Locked", goalTargeter.isLocked() ? "YES ✓" : "NO");
        }
        telemetry.addLine();

        // Color Detection
        telemetry.addData("Color Detection", "---");
        if (visionData.hasTarget()) {
            if (visionData.isGreen()) {
                telemetry.addData("  Artifact", "🟢 GREEN");
            } else if (visionData.isPurple()) {
                telemetry.addData("  Artifact", "🟣 PURPLE");
            } else {
                telemetry.addData("  Artifact", "None (not color pipeline)");
            }
        }
        telemetry.addLine();

        // MotifDetector data
        telemetry.addData("Motif", motifDetector.getDetectedMotif().toString());
        if (motifDetector.hasConfidentDetection()) {
            telemetry.addData("  Pattern", motifDetector.getPatternDescription());
            telemetry.addData("  Confidence", "%d detections", motifDetector.getConsecutiveDetections());
        }
        telemetry.addLine();

        // Drive corrections (for debugging)
        if (autoAlignEnabled) {
            telemetry.addData("Steer Correction", "%.2f", goalTargeter.getSteeringCorrection());
            telemetry.addData("Drive Correction", "%.2f", goalTargeter.getDriveCorrection());
        }
    }

    @Override
    public void stop() {
        drive.drive(0, 0, 0);
    }
}
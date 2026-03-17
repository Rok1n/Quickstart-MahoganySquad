package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDFflywheel extends OpMode {

    public DcMotorEx flywheelmotor;

    public double highVelocity = 1280;
    public double lowVelocity = 1025;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init(){
        flywheelmotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelmotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete.");
    }

    @Override
    public void loop(){

        if(gamepad1.yWasPressed()) {
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

       PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelmotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //flywheelmotor.setVelocityPIDFCoefficients(P, 0, 0, F);

        flywheelmotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelmotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);


        telemetry.addData("Error", "%.2f", error);

        telemetry.addData("Tuning P", "%.4f", P);
        telemetry.addData("Tuning F", "%.4f", F);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.update();
    }
}
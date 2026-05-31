package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Diagnostic", group = "Sensor")
public class ColorSensorDiagnostic extends LinearOpMode {
    private static final float DEFAULT_GAIN = 10.0f;
    private static final float GAIN_STEP = 1.0f;
    private static final double TOGGLE_DEBOUNCE_S = 0.25;

    @Override
    public void runOpMode() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        float gain = DEFAULT_GAIN;
        boolean lightEnabled = false;
        double nextToggleTime = 0.0;

        colorSensor.setGain(gain);
        if (colorSensor instanceof SwitchableLight) {
            lightEnabled = true;
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Config name: sensor_color");
        telemetry.addLine("Controls: D-pad up/down adjusts gain, A toggles light");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                gain += GAIN_STEP;
            } else if (gamepad1.dpad_down) {
                gain = Math.max(1.0f, gain - GAIN_STEP);
            }
            colorSensor.setGain(gain);

            if (gamepad1.a && colorSensor instanceof SwitchableLight && getRuntime() > nextToggleTime) {
                lightEnabled = !lightEnabled;
                ((SwitchableLight) colorSensor).enableLight(lightEnabled);
                nextToggleTime = getRuntime() + TOGGLE_DEBOUNCE_S;
            }

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addData("Gain", "%.1f", gain);
            telemetry.addData("Light", lightEnabled ? "on" : "off");
            telemetry.addData("RGBA", "%.3f, %.3f, %.3f, %.3f",
                    colors.red, colors.green, colors.blue, colors.alpha);
            telemetry.addData("HSV", "%.1f, %.3f, %.3f",
                    hsvValues[0], hsvValues[1], hsvValues[2]);

            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.2f",
                        ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.update();
        }
    }
}

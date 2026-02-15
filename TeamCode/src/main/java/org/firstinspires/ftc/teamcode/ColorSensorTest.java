package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp
public class ColorSensorTest extends OpMode {

    NormalizedColorSensor colorSensor;
    DetectedColor detectedColor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        YELLOW,
        UNKNOWN
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        mapHardware();

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        detectedColor = getDetectedColor();
        telemetry.addData("Detected Color", detectedColor);
    }

    private void mapHardware() {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        // (Optional) Set Gain - higher number = more sensitive to dark colors (like Purple)
        colorSensor.setGain(2);
    }

    public DetectedColor getDetectedColor() {

        // Use a float array to hold the HSV values
        float[] hsv = new float[3];

        // 1. Get the normalized colors (0.0 to 1.0)
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // Return 4 values

        // 2. Convert to HSV
        // colorSensor.toColor() converts the normalized values to a single Android Color int
        Color.colorToHSV(colors.toColor(), hsv);

        // 3. Extract the Hue (0 - 360)
        float hue = hsv[0];
        float saturation = hsv[1]; // Vividness - how "intense" the color is
        float value = hsv[2]; // Brightness - how much light is hitting the sensor

        //-- Uncomment this code only for calibration
        /*
        telemetry.addLine("--- SENSOR DATA ---");
        telemetry.addData("HUE (Color)", "%.2f", hue);
        telemetry.addData("SAT (Intensity)", "%.2f", saturation);
        telemetry.addData("VAL (Brightness)", "%.2f", value);

        telemetry.addLine("--- RGB NORMALIZED ---");
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        telemetry.update();
        */

        // Use a 'Gatekeeper' value so it doesn't detect the floor (Ambient 0.00)
        // We'll use 0.01 as a safe starting point
        if (value < 0.01) {
            return DetectedColor.UNKNOWN;
        }

        if (hue >= 120 && hue < 180) {
            return DetectedColor.GREEN;
        } else if (hue >= 180 && hue <= 240) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
}

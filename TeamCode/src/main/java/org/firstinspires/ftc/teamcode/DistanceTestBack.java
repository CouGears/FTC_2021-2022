package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class DistanceTestBack extends LinearOpMode {
    private DistanceSensor distanceSensorBack;
    private ColorSensor sensorColor;

    @Override
    public void runOpMode() {

        distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");

        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

           telemetry.addData("Distance: ", distanceSensorBack.getDistance(DistanceUnit.CM));
           telemetry.update();
//            telemetry.addData("Distance (cm)",
        /*    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            telemetry.addData("Alpha", sensorColor.alpha());
          telemetry.addData("Red  ", sensorColopackage org.firstinspires.ftc.teamcode;*/
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.graphics.Color;

@TeleOp

public class DistanceTest extends LinearOpMode {
    private DistanceSensor distanceSensor;
    private ColorSensor sensorColor;

    @Override
    public void runOpMode() {

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

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

           telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
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

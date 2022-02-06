package javaold.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import javaold.Odometry.OdometryMethods;

@TeleOp

public class LEDTest extends LinearOpMode {
    public static LED red, green, red2, green2, red3, green3, red4, green4;
    private OdometryMethods robot = new OdometryMethods();


    @Override
    public void runOpMode() {
        red = hardwareMap.get(LED.class, "red");
        green = hardwareMap.get(LED.class, "green");
        red2 = hardwareMap.get(LED.class, "red");
        green2 = hardwareMap.get(LED.class, "green");
        red3 = hardwareMap.get(LED.class, "red");
        green3 = hardwareMap.get(LED.class, "green");
        red4 = hardwareMap.get(LED.class, "red");
        green4 = hardwareMap.get(LED.class, "green");
//      intakeServo.setPosition(1);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("red", 0);
           telemetry.update();
            setRed(1);
            telemetry.addData("amber", 0);
            telemetry.update();
            setAmber(1);
           telemetry.addData("green", 0);
          telemetry.update();
            setGreen(1);
        }
    }
    public void setRed(double delay)
    {
        red.enable(true);
        red2.enable(true);
        red3.enable(true);
        red4.enable(true);
        robot.newSleep(delay);
        red.enable(false);
        red2.enable(false);
        red3.enable(false);
        red4.enable(false);
        robot.newSleep(delay);
    }
    public void setGreen(double delay)
    {
        green.enable(true);
        green2.enable(true);
        green3.enable(true);
        green4.enable(true);
        robot.newSleep(delay);
        green.enable(false);
        green2.enable(false);
        green3.enable(false);
        green4.enable(false);
        robot.newSleep(delay);
    }
    public void setAmber(double delay)
    {
        red.enable(true);
        red2.enable(true);
        red3.enable(true);
        red4.enable(true);
        green.enable(true);
        green2.enable(true);
        green3.enable(true);
        green4.enable(true);
        robot.newSleep(delay);
        red.enable(false);
        red2.enable(false);
        red3.enable(false);
        red4.enable(false);
        green.enable(false);
        green2.enable(false);
        green3.enable(false);
        green4.enable(false);
        robot.newSleep(delay);
    }
}

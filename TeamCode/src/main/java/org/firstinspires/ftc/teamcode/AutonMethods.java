package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;
import android.app.Activity;
import android.view.View;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;
import java.util.Timer;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import android.app.Activity;

public class AutonMethods {

    //Constructor
    public AutonMethods() {

    }

    //Declare and initial variables
    double rev = 383.6;
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12;
    double FRtpos, BRtpos, FLtpos, BLtpos;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, shooter, arm, scissorMotor;
    private static Servo shooterServo, armServo, marker, frontScissor, backScissor;
    public static Servo armBlock;
    public static DistanceSensor topSensor, bottomSensor, sensorDistance;
    public TouchSensor armTouch, scissorTouch;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;

    private double speed;

    public int counter = 0;


    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        intakeFL = map.get(DcMotor.class, "intake");
        arm = map.get(DcMotor.class, "arm");
        shooter = map.get(DcMotor.class, "shooter");


        armServo = map.get(Servo.class, "armServo");
        shooterServo = map.get(Servo.class, "shooterServo");
        //note - this is according to front orientation - front is in the front and back is in the back
        //also these should be configured accordingly
        scissorMotor = map.get(DcMotor.class, "scissorMotor");
        armBlock = map.get(Servo.class, "armBlock");
        bottomSensor = map.get(DistanceSensor.class, "bottomSensor");
        topSensor = map.get(DistanceSensor.class, "sensor_color_distance");
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        scissorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);


        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        scissorMotor.setTargetPosition(0);
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }

    //Function to move the robot in any direction
    public void drive(double forward, double sideways, double spee) {
        runtime.reset();
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRtpos = forward - sideways;
        BRtpos = forward + sideways;
        FLtpos = forward + sideways;
        BLtpos = forward - sideways;

        motorFL.setTargetPosition((int) FLtpos);
        motorBL.setTargetPosition((int) BLtpos);
        motorFR.setTargetPosition(-(int) FRtpos);
        motorBR.setTargetPosition(-(int) BRtpos);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed(spee);


    }
    //circumscibed robot has a diameter of 21 inches
    public void turn(double deg) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //for every drive function remember to reset encoder
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double deltaturn = (deg/360.0)*21.654*3.14*inch*1.5;
        motorFL.setTargetPosition((int)  deltaturn);
        motorBL.setTargetPosition((int)  deltaturn);
        motorFR.setTargetPosition((int)  deltaturn);
        motorBR.setTargetPosition((int)  deltaturn);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setPower(0.5);
        motorBL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBR.setPower(0.5);

    }

    public void armTime(Double time, int dir) {
        arm.setPower(dir);
        runtime.reset();
        while (runtime.seconds() < time) ;
        arm.setPower(0);
    }

    public void scissorServUp() {

        sleep(150);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setTargetPosition(-3150);
        scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scissorMotor.setPower(1);
        sleep(500);

    }

    public void scissorServDown() {
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setTargetPosition(0);
        scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scissorMotor.setPower(1);

        sleep(500);
    }


    public void shootServ(double pos) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        shooterServo.setPosition(pos);


    }

    public void armServ(double pos) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 3) break;
        }
        armServo.setPosition(pos);

    }

    public void arm(int pos) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 3) break;
        }
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

    }

    public void shoot(double power) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 3) break;
        }
        shooter.setPower(power);

    }

    public int distance() {
        int rings = 0;
        if (bottomSensor.getDistance(DistanceUnit.CM) < 14 && topSensor.getDistance(DistanceUnit.CM) > 20) {
            // tele.addData("One ring", bottomSensor.getDistance(DistanceUnit.CM));
            // tele.update();
            rings = 1;
        } else if (bottomSensor.getDistance(DistanceUnit.CM) < 14 && topSensor.getDistance(DistanceUnit.CM) < 20) {
            // tele.addData("Four rings", topSensor.getDistance(DistanceUnit.CM));
            // tele.update();
            rings = 4;
        } else {
            // tele.addData("No rings", bottomSensor.getDistance(DistanceUnit.CM));
            // tele.update();
            rings = 0;
        }
        return rings;
    }


    public void intake(int time) {
        intakeFL.setPower(1);
        sleep(time);
        intakeFL.setPower(0);

    }

    public void topGoal() {
        runtime.reset();
        while (runtime.seconds() < 3) ;
        shootServ(.2);
        runtime.reset();
        while (runtime.seconds() < 0.5) ;
        shootServ(0);
        // runtime.reset();
        // while (runtime.seconds() < 1) ;
    }


    //Function to have the robot sleep
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }


    }

    public void driveWithDecel(double forward, double sideways) {
        double dist = 8 * feet;
        runtime.reset();
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 3) break;
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRtpos = forward - sideways;
        BRtpos = forward + sideways;
        FLtpos = forward + sideways;
        BLtpos = forward - sideways;

        motorFL.setTargetPosition((int) FLtpos);
        motorBL.setTargetPosition((int) BLtpos);
        motorFR.setTargetPosition(-(int) FRtpos);
        motorBR.setTargetPosition(-(int) BRtpos);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed(1);
        runtime.reset();
        while ((motorFR.isBusy() || motorFL.isBusy()) && runtime.seconds() < 3) {
            motorFL.setPower((((int) FLtpos - motorFL.getCurrentPosition()) / dist) + .2);
            motorBL.setPower((((int) BLtpos - motorBL.getCurrentPosition()) / dist) + .2);
            motorFR.setPower((((int) FRtpos + motorFR.getCurrentPosition()) / dist) + .2);
            motorBR.setPower((((int) BRtpos + motorBR.getCurrentPosition()) / dist) + .2);
        }


    }

    public void speed(double spee) {
        motorFL.setPower(spee);
        motorBL.setPower(spee);
        motorFR.setPower(spee);
        motorBR.setPower(spee);
    }

    public void newSleep(double timeinSeconds) {
        runtime.reset();
        while (runtime.seconds() < timeinSeconds) ;
    }

}

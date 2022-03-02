package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonMethods {

    //Constructor
    public AutonMethods() {

    }

    //Declare and initial variables
    double rev = 537.7;
    double inch = rev / (3.5 * 3.14);
    double feet = inch * 12;
    double rev2 = 2048;
    double inch2 = rev2 / (2 * 3.14);
    double feet2 = inch2 * 12;
    double FRtpos, BRtpos, FLtpos, BLtpos;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, lifter, intake, carousel;
    //public static DcMotor Forwards = intake, Sideways = carousel;
    public static Servo bucket, intakeServo, liftyThingy;
    public static DistanceSensor distanceSensor, distanceSensorBack;
    public TouchSensor armTouch;
    private final ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;
    double locationx = 0;
    double locationy = 0;
    private double speed;
    public int crap = 0;
    public int counter = 0;
    public double dist;
    public double distBack;
    public int BlockPosition = 1;
    public static LED red, green, red2, green2;


    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        // location[0] = 0;
        //location[1] = 0;

        distanceSensor = map.get(DistanceSensor.class, "distanceSensor");
        distanceSensorBack = map.get(DistanceSensor.class, "distanceSensorBack");
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        intake = map.get(DcMotor.class, "intake");
        lifter = map.get(DcMotor.class, "lifter");

        red = map.get(LED.class, "red");
        green = map.get(LED.class, "green");
        red2 = map.get(LED.class, "red2");
        green2 = map.get(LED.class, "green2");

        carousel = map.get(DcMotor.class, "carousel");
        //   claw1 = hardwareMap.get(Servo.class, "claw1");
        // claw2 = hardwareMap.get(Servo.class, "claw2");

        bucket = map.get(Servo.class, "bucket");
        liftyThingy = map.get(Servo.class, "liftyThingy");
        intakeServo = map.get(Servo.class, "serv");


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }

    public void kill() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }
public void setServo()
{
    liftyThingy.setPosition(1);
}
    public void setRed2()
    {
        green2.enable(false);
        red2.enable(true);
    }
    public void setGreen2()
    {
        red2.enable(false);
        green2.enable(true);
    }
    public void setAmber2()
    {
        red2.enable(true);
        green2.enable(true);
    }
    public void setRed()
    {
        green.enable(false);
        red.enable(true);
    }
        public void setGreen()
        {
            red.enable(false);
            green.enable(true);
        }
        public void setAmber()
        {
            red.enable(true);
            red.enable(true);
            green.enable(true);
        }
public int value() {return 0;}

    public double distanceSet() {
        dist = distanceSensor.getDistance(DistanceUnit.CM);
        return dist;
    }
    public double distanceSetBack() {
        distBack = distanceSensorBack.getDistance(DistanceUnit.CM);
        return distBack;
    }
    public long maps(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
public void blockPosBlue()
{
    if(BlockPosition == 1)
    {
        drive(-9*inch,0,.5);
        lift(1500);
    }
    else if (BlockPosition == 2)
    {
        drive(-9*inch,0,.5);
        lift(2500);
    }
    else if (BlockPosition == 3)
    {
        drive(-10*inch,0,.5);
        lift(3300);
    }
}
    public void blockPosRed()
    {
        if(BlockPosition == 3)
        {
            drive(-15*inch,0,.5);
            lift(1500);
        }
        else if (BlockPosition == 2)
        {
            drive(-15*inch,0,.5);
            lift(2500);
        }
        else if (BlockPosition == 1)
        {
            drive(-15*inch,0,.5);
            lift(3300);
        }
    }
public void autonLowerBlue()
{
    if(BlockPosition == 1)
    {
        lift(-1500);
    }
    else if (BlockPosition == 2)
    {
        lift(-2500);
    }
    else if (BlockPosition == 3)
    {
        lift(-3300);
    }
}
    public void autonLowerRed()
    {
        if(BlockPosition == 3)
        {
            lift(-1500);
        }
        else if (BlockPosition == 2)
        {
            lift(-2500);
        }
        else if (BlockPosition == 1)
        {
            lift(-3300);
        }
    }
    //Function to move the robot in any direction
    public void drive(double forward, double sideways, double speed) {
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
        FLtpos = forward - sideways;
        BLtpos = forward + sideways;

        motorFL.setTargetPosition(-(int) FLtpos);
        motorBL.setTargetPosition((int) BLtpos);
        motorFR.setTargetPosition(-(int) FRtpos);
        motorBR.setTargetPosition((int) BRtpos);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed(speed);


    }

    public void setIntakeServo() {
        intakeServo.setPosition(.45);
    }

    public void setCarousel(double pwr) {
        carousel.setPower(pwr);
        newSleep(3);
        carousel.setPower(0);
    }

    public int distance() {
        int stuff = 3300;
        if (dist < 25) {
            stuff = 3300;
            crap = 3300;
            setRed();
            setRed2();
        } else if (dist >= 25 && dist < 40) {
            stuff = 2000;
            crap = 2000;
            setAmber();
            setAmber2();
        } else if (dist >= 40) {
            stuff = 1500;
            crap = 1500;
            setGreen2();
            setGreen();
        }
        return stuff;
    }

    //circumscribed robot has a diameter of 21 inches
    public void turn(double deg) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //for every drive function remember to reset encoder
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double deltaturn = (deg / 360.0) * 21.654 * 3.14 * inch * 2.2;
        motorFL.setTargetPosition(-(int) deltaturn);
        motorBL.setTargetPosition((int) deltaturn);
        motorFR.setTargetPosition((int) deltaturn);
        motorBR.setTargetPosition(-(int) deltaturn);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setPower(0.5);
        motorBL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBR.setPower(0.5);

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

    //Function to have the robot sleep
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }
    }

    public void lift(double amount) { //moves the 4 bar/lifter
       // amount = -amount;
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setTargetPosition((int) amount);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(.6);
    }

    public void dump() {

        bucket.setPosition(.05);
        sleep(2000);
        bucket.setPosition(.49);
    }

    public void setIntake(int pos) {
        intakeServo.setPosition(pos);
    }
    public void driveWithDecel(double forward, double sideways) {
        double distd = 8 * feet;
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
            motorFL.setPower((((int) FLtpos - motorFL.getCurrentPosition()) / distd) + .2);
            motorBL.setPower((((int) BLtpos - motorBL.getCurrentPosition()) / distd) + .2);
            motorFR.setPower(-(((int) FRtpos + motorFR.getCurrentPosition()) / distd) + .2);
            motorBR.setPower(-(((int) BRtpos + motorBR.getCurrentPosition()) / distd) + .2);
        }


    }


}

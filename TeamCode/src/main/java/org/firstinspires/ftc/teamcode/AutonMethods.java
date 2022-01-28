package org.firstinspires.ftc.teamcode;

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
    public static DcMotor motorBR, motorBL, motorFL, motorFR, rum, intake, carousel;
    //public static DcMotor Forwards = intake, Sideways = carousel;
    public static Servo bucket, intakeServo;
    public static DistanceSensor distanceSensor;
    public TouchSensor armTouch;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;
    double locationx = 0;
    double locationy = 0;
    private double speed;

    public int counter = 0;
    public double dist;

    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        // location[0] = 0;
        //location[1] = 0;

        distanceSensor = map.get(DistanceSensor.class, "distanceSensor");
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        intake = map.get(DcMotor.class, "intake");
        rum = map.get(DcMotor.class, "lifter");

        carousel = map.get(DcMotor.class, "carousel");
        //   claw1 = hardwareMap.get(Servo.class, "claw1");
        // claw2 = hardwareMap.get(Servo.class, "claw2");

        bucket = map.get(Servo.class, "bucket");
        intakeServo = map.get(Servo.class, "intakeServo");



        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rum.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        rum.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }

public void kill()
{
    motorFL.setPower(0);
    motorBL.setPower(0);
    motorBR.setPower(0);
    motorFR.setPower(0);
}
public void distanceSet()
{
    dist = distanceSensor.getDistance(DistanceUnit.CM);
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



    public void setCarousel(double pwr) {
        carousel.setPower(pwr);
        newSleep(4);
        carousel.setPower(0);
    }

    public int distance() {
        int stuff = 3300;
        if (dist < 20) {
            stuff = 3300;
        } else if (dist > 25 && dist < 40) {
            stuff = 2500;
        } else if (dist > 45) {
            stuff = 2000;
        }
        return stuff;
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
        double deltaturn = (deg / 360.0) * 21.654 * 3.14 * inch * 2.2 ;
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
        amount = -amount;
        rum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rum.setTargetPosition((int) amount);
        rum.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rum.setPower(.6);
    }

    public void dump() {

        bucket.setPosition(0);
        sleep(1500);
        bucket.setPosition(.5);
    }

    /*


     public void intake(int time) {
         intakeFL.setPower(1);
         sleep(time);
         intakeFL.setPower(0);

     }






     }
 */
    public void setIntake(int pos)
    {
        intakeServo.setPosition(pos);
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
            motorFR.setPower(-(((int) FRtpos + motorFR.getCurrentPosition()) / dist) + .2);
            motorBR.setPower(-(((int) BRtpos + motorBR.getCurrentPosition()) / dist) + .2);
        }


    }


}

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.*;

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
    double rev2 = 2048;
    double inch2 = rev2 / (2 * 3.14);
    double feet2 = inch2 * 12;
    double FRtpos, BRtpos, FLtpos, BLtpos;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, arm, rum, intake, carousel, FODO, SODO; //rum refers to the 4-bar
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
        //SODO = map.get(DcMotor.class, "SODO");
        //FODO = map.get(DcMotor.class, "FODO");
        carousel = map.get(DcMotor.class, "carousel");
        rum = map.get(DcMotor.class, "4-bar");
        bucket = map.get(Servo.class, "bucket");
        //intakeServo = map.get(Servo.class, "intakeServo");
        intake = map.get(DcMotor.class, "intake");
//        arm = map.get(DcMotor.class, "arm");

        //  shooter = map.get(DcMotor.class, "shooter");


        // armServo = map.get(Servo.class, "armServo");
        // shooterServo = map.get(Servo.class, "shooterServo");
        //   note - this is according to front orientation - front is in the front and back is in the back
        // also these should be configured accordingly


        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   FODO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //    SODO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // FODO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //SODO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // FODO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //SODO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
       // FODO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //SODO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
     //   FODO.setTargetPosition(0);
       // SODO.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }


  /*  public void getLocation() {
          locationy = (FODO.getCurrentPosition() / feet2);
          locationx = (SODO.getCurrentPosition() / feet2);
    }*/
public void kill()
{
    motorFL.setPower(0);
    motorBL.setPower(0);
    motorBR.setPower(0);
    motorFR.setPower(0);
}
   /* public void movefb(double forwards) {
        getLocation();
        if (forwards > 0) {
            while (motorFR.isBusy() || motorFL.isBusy()) {
                if (runtime.seconds() > 3) break;//do i need to runtime reset?
            }
            while (forwards - locationy >= 1) {
                getLocation();//do i need to runtime reset?
                motorFL.setPower(.5);
                motorBL.setPower(.5);
                motorBR.setPower(.5);
                motorFR.setPower(.5);
                //drive until robot.locationy>movefb
                tele.addData("move forwards", .5);
                tele.update();//do i need to runtime reset?
            }
            while (forwards - locationy < 1) {
                getLocation();//do i need to runtime reset?
                motorFL.setPower(.1);//do i need to runtime reset?
                motorBL.setPower(.1);//do i need to runtime reset?
                motorBR.setPower(.1);
                motorFR.setPower(.1);
                //drive until robot.locationy>movefb
                tele.addData("move forwards", .5);
                tele.update();//do i need to runtime reset?
            }
        }
        else if (forwards < 0) {
            while (motorFR.isBusy() || motorFL.isBusy()) {
                if (runtime.seconds() > 3) break;
            }
            while (locationy > forwards) {
                getLocation();
                motorFL.setPower(-.5);
                motorBL.setPower(-.5);
                motorBR.setPower(-.5);
                motorFR.setPower(-.5);
                //drive until robot.locationy<movefb
                tele.addData("move backwqards", .5);
                tele.update();
            }
        }
        else kill();
    }

    public void moverl(double sideways) {
        getLocation();
        if (sideways > 0) {
            if (locationx < sideways) {
                while (motorFR.isBusy() || motorFL.isBusy()) {
                    if (runtime.seconds() > 3) break;
                }
                getLocation();
                motorFL.setPower(.5);
                motorBL.setPower(-.5);
                motorBR.setPower(.5);
                motorFR.setPower(-.5);
                //drive until robot.locationx>moverl
                tele.addData("move right", .5);
                tele.update();
            }
        }
        else if (sideways < 0) {
            if (locationx > sideways) {
                while (motorFR.isBusy() || motorFL.isBusy()) {
                    if (runtime.seconds() > 3) break;
                }
                getLocation();
                motorFL.setPower(-.5);
                motorBL.setPower(.5);
                motorBR.setPower(-.5);
                motorFR.setPower(.5);
                //drive until robot.locationx<moverl
                tele.addData("move left", .5);
                tele.update();
            }
        }
        else kill();
    }
*/

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
        FLtpos = forward + sideways;
        BLtpos = forward - sideways;

        motorFL.setTargetPosition(-(int) FLtpos);
        motorBL.setTargetPosition(-(int) BLtpos);
        motorFR.setTargetPosition(-(int) FRtpos);
        motorBR.setTargetPosition((int) BRtpos);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed(speed);


    }

    public void setCarousel() {
        carousel.setPower(.45);
        newSleep(4);
        carousel.setPower(0);
    }

    public int distance() {
        int stuff = 0;
        if (distanceSensor.getDistance(DistanceUnit.CM) < 5) {
            stuff = 1;
        } else if (distanceSensor.getDistance(DistanceUnit.CM) > 10 && distanceSensor.getDistance(DistanceUnit.CM) < 15) {
            stuff = 2;
        } else if (distanceSensor.getDistance(DistanceUnit.CM) > 15) {
            stuff = 3;
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
        double deltaturn = (deg / 360.0) * 21.654 * 3.14 * inch * 1.5;
        motorFL.setTargetPosition((int) deltaturn);
        motorBL.setTargetPosition((int) deltaturn);
        motorFR.setTargetPosition((int) deltaturn);
        motorBR.setTargetPosition((int) deltaturn);
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

    public void alcohol(double tequila) { //moves the 4 bar/arm
        rum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rum.setTargetPosition((int) tequila);
        rum.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lift() {
        bucket.setPosition(.5);
        sleep(1000);
        bucket.setPosition(1);
    }

    /*


     public void intake(int time) {
         intakeFL.setPower(1);
         sleep(time);
         intakeFL.setPower(0);

     }






     }
 */
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


}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class CompetitionDriving2021 extends LinearOpMode {

    private DcMotor motorBR, motorBL, motorFL, motorFR, intake, lifter, carousel, lift;
    private Servo bucket, intakeServo, liftyThingy;//, hServo, vServo;
    //private CRServo dServo;
    private boolean claw = false, bucketButton = false;
    private double switch1Smoothed, switch1Prev;
    private AutonMethods robot = new AutonMethods();
    int x = 0;
    int SWITCH = 0;
    double xtape = .5;
    double ytape = .5;
    double extendpower = 0;
    double pextend = .2;
    // double 

    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        lift = hardwareMap.get(DcMotor.class, "lift");

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //   claw1 = hardwareMap.get(Servo.class, "claw1");
        // claw2 = hardwareMap.get(Servo.class, "claw2");

        bucket = hardwareMap.get(Servo.class, "bucket");
        intakeServo = hardwareMap.get(Servo.class, "serv");
        liftyThingy = hardwareMap.get(Servo.class, "liftyThingy");

       // hServo = hardwareMap.get(Servo.class, "hServo");
       // vServo = hardwareMap.get(Servo.class, "vServo");
      //  dServo = hardwareMap.crservo.get("dServo");



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

        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

//      intakeServo.setPosition(1);
        waitForStart();

        while (opModeIsActive()) {
            x = 0;
        intakeServo.setPosition(.45);

            if (x == 0) {
                motorFL.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x)) * .9);
                motorBL.setPower((-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * .9);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * .9);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * .9);
                /*
                 motorFL.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x)) * .9);
                motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * .9);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * .9);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * .9);
                */

            } else if (x == 1) {
                motorFL.setPower(((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x)) * .25);
                motorBL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x)) * .25);
                motorBR.setPower(-((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x)) * .25);
                motorFR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x)) * .25);
            }


            if (gamepad1.b) {
                bucket.setPosition(0);
                robot.sleep(1500);
                bucket.setPosition(.5);
                telemetry.addData("Position:", .5);
                telemetry.update();
            }
            else {
                bucket.setPosition(.5);
            }
            int FLIntakePowerR = (int) gamepad1.right_trigger;
            boolean FLIntakePowerL = gamepad1.left_bumper;

            if (gamepad1.right_bumper) {
                intake.setPower(-1);
            } else if (FLIntakePowerL) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.dpad_up)lifter.setPower(.8);
            else if (gamepad1.dpad_down) lifter.setPower(-.8);
            else lifter.setPower(0);


            if (gamepad1.y)lift.setPower(.6);
            else if (gamepad1.a)lift.setPower(-.6);
            else lift.setPower(0);

            if (gamepad1.x) {
                if (SWITCH == 0) {
                    robot.newSleep(.5);
                    SWITCH++;
                } else if (SWITCH == 1) {
                    robot.newSleep(.5);
                    SWITCH--;
                }
            }
            if (SWITCH==0)
            {
                liftyThingy.setPosition(1-(gamepad1.right_trigger*.33));
            }
            if (SWITCH ==1)
            {
                liftyThingy.setPosition(.66+(gamepad1.right_trigger*.33));
            }

            if (gamepad1.dpad_left) {
                switch1Smoothed = ((1*.005) + (switch1Prev * .995));
                switch1Prev = switch1Smoothed;
                telemetry.addData("speed", switch1Smoothed);
                telemetry.update();
                carousel.setPower(switch1Smoothed);
            }
            else if (gamepad1.dpad_right) {
                switch1Smoothed = ((1*.005) + (switch1Prev * .995));
                switch1Prev = switch1Smoothed;
                carousel.setPower(-switch1Smoothed);
            }
            else carousel.setPower(0);//set ===to while else??
            
            if (xtape <= .97 && xtape >= -.97) xtape = xtape + this.gamepad2.right_stick_x * .03;
            if (ytape <= .97 && ytape >= -.97) ytape = ytape + this.gamepad2.right_stick_y * .03;

            if (gamepad2.right_bumper)extendpower = pextend;
            else if (gamepad2.right_bumper)extendpower = -pextend;
            else extendpower = 0;
            //hServo.setPosition(ytape);
            //vServo.setPosition(xtape);
            //dServo.setPower(extendpower);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class CompetitionDriving2021 extends LinearOpMode {

    private DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, liftR, liftL, carousel;
    private Servo bucketR, bucketL, intakeServo;
    private boolean claw = false, bucketButton = false;
    private AutonMethods robot = new AutonMethods();
    int x = 0;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intakeFL = hardwareMap.get(DcMotor.class, "intake");
        liftR = hardwareMap.get(DcMotor.class, "liftR");
        liftL = hardwareMap.get(DcMotor.class, "liftR");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //   claw1 = hardwareMap.get(Servo.class, "claw1");
        // claw2 = hardwareMap.get(Servo.class, "claw2");
        bucketR = hardwareMap.get(Servo.class, "bucketR");
        bucketL = hardwareMap.get(Servo.class, "bucketL");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");



        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.FORWARD);
        liftL.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        intakeServo.setPosition(1);
        waitForStart();

        while (opModeIsActive()) {
            x = 0;
        intakeServo.setPosition(0);


            if (x == 0) {
                motorFL.setPower(((this.gamepad1.right_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.left_stick_x)) * .9);
                motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * .9);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * .9);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * .9);
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
           /* if(gamepad1.a) {
                if (claw == false) {
                    claw1.setPosition(.5);
                    claw2.setPosition(.5);
                    claw = !claw;
                } else {
                    claw1.setPosition(1);
                    claw2.setPosition(1);
                    claw = !claw;
                }
            }*/
          /*  while (gamepad1.b){

                bucket.setPosition(.5);
                robot.sleep(1000);
               // bucket.setPosition(1);
                bucket.setPosition(-.5);//to turn the servo backwards
                //bucketButton = !bucketButton;
            }*/
         /*   if (gamepad1.b) {
                bucket.setPosition(.5);
                robot.sleep(500);
                bucket.setPosition(0);

            }*/
            if (gamepad1.b) {
                bucket.setPosition(.5);
                robot.sleep(1500);
                bucket.setPosition(1);
            } else if (gamepad1.y){
                bucket.setPosition(1);
            }

            if (gamepad1.a) {
                intakeServo.setPosition(0);
            }
/*
            if(gamepad1.right_bumper) {
                intakeFL.setPower(1);
            } else if(gamepad1.left_bumper){
                intakeFL.setPower(-1);
            } else {
                intakeFL.setPower(0);
            }
*/
 /*
  public int FLIntakePowerR = gamepad1.right_trigger;
  public int FLIntakePowerL = gamepad1.left_bumper;
*/
            int FLIntakePowerR = (int) gamepad1.right_trigger;
            boolean FLIntakePowerL = gamepad1.left_bumper;

            if (gamepad1.right_bumper) {
                intakeFL.setPower(-1);
            } else if (FLIntakePowerL) {
                intakeFL.setPower(1);
            } else {
                intakeFL.setPower(0);
            }

//hello there
            if (gamepad1.dpad_up) {
                liftR.setPower(.5);
                liftL.setPower(.5);
            } else if (gamepad1.dpad_down) {
                liftR.setPower(-.5);
                liftL.setPower(-.5);
            } else {
                liftR.setPower(0);
                liftL.setPower(0);
            }

            if (gamepad1.x) {
                carousel.setPower(.45);
                //robot.sleep(2000);
                //carousel.setPower(0);
            } else if (gamepad1.y) carousel.setPower(-.45);
            else carousel.setPower(0);//set to while else??
        }
    }
}


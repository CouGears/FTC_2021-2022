package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class CompetitionDriving2021 extends LinearOpMode {

    private DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, lifter, carousel;
    //private Servo claw1, claw2,
    private Servo bucket, intakeservo;
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
        lifter = hardwareMap.get(DcMotor.class, "4-bar");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //   claw1 = hardwareMap.get(Servo.class, "claw1");
        // claw2 = hardwareMap.get(Servo.class, "claw2");
        bucket = hardwareMap.get(Servo.class, "bucket");
        intakeservo = hardwareMap.get(Servo.class, "intakeservo");


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intakeservo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            x = 0;

            intakeservo.setPosition(1);


            if (x == 0) {
                motorFL.setPower((-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x)) * .9);
                motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x)) * .9);
                motorBR.setPower(-(-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x)) * .9);
                motorFR.setPower((-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x)) * .9);
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
            } else if (gamepad1.y){
                bucket.setPosition(1);
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
                intakeFL.setPower(FLIntakePowerR);
            } else if (FLIntakePowerL) {
                intakeFL.setPower(-1);
            } else {
                intakeFL.setPower(0);
            }


            if (gamepad1.dpad_up) {
                lifter.setPower(.5);
            } else if (gamepad1.dpad_down) {
                lifter.setPower(-.5);
            } else {
                lifter.setPower(0);
            }
            if (gamepad1.x) {
                carousel.setPower(1);
                //robot.sleep(2000);
                //carousel.setPower(0);
            }
        }
    }
}


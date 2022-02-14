package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class CompetitionDriving2021 extends LinearOpMode {

    private DcMotor motorBR, motorBL, motorFL, motorFR, intake, lifter, carousel, lift;
    private Servo bucket, intakeServo, liftyThingy;
    private boolean claw = false, bucketButton = false;
    private double switch1Smoothed, switch1Prev;
    private AutonMethods robot = new AutonMethods();
    int x = 0;
    int SWITCH = 0;

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

//hello there
            if (gamepad1.dpad_up) {
                lifter.setPower(-.6);

            } else if (gamepad1.dpad_down) {
                lifter.setPower(.6);

            } else {
                lifter.setPower(0);

            }
            if (gamepad1.y) {

                lift.setPower(.6);

            } else if (gamepad1.a) {
                lift.setPower(-.6);

            } else {
                lift.setPower(0);

            }
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
          /*  */

            if (gamepad1.dpad_left) {
                /*for (int i = 0; i < .7; i+=.1){
                    opModeIsActive();
                    carousel.setPower(i);
                    telemetry.addData("speed", i);
                    telemetry.update();
//                    robot.newSleep(.1);
                }*/

                switch1Smoothed = ((1*.005) + (switch1Prev * .995));
                switch1Prev = switch1Smoothed;
                telemetry.addData("speed", switch1Smoothed);
                telemetry.update();
                carousel.setPower(switch1Smoothed);
                //robot.sleep(2000);
                //carousel.setPower(0);
            } else if (gamepad1.dpad_right){
                switch1Smoothed = ((1*.005) + (switch1Prev * .995));
                switch1Prev = switch1Smoothed;
                carousel.setPower(-switch1Smoothed);
            }
            else {
                switch1Smoothed = 0;
                switch1Prev = 0;
                telemetry.addData("speed", 0);
                carousel.setPower(0);//set ===to while else??
            }

        }
    }
}


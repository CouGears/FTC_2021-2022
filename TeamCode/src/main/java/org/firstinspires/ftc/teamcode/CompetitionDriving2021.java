package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
//Hi Oran
public class CompetitionDriving2021 extends LinearOpMode {

    private DcMotor motorBR, motorBL, motorFL, motorFR, intake, lifter, carousel, capDrive;
    private Servo bucket, intakeServo, liftyThingy;//, hServo, vServo;
    private Servo capServo;
    private boolean claw = false, bucketButton = false;
    private double switch1Smoothed, switch1Prev;
    private AutonMethods robot = new AutonMethods();
    int x = 0;
    int SWITCH = 0;
    double xtape = .5;
    double ytape = .5;
    double extendpower = 0;
    double pextend = .2;
    double degree = 3.9586;
    double liftArmPos = 0;

    double ticks = 1124; // ticks for cap motor; half rotation of arm
    double pos = 0; // overall cap position
    double alpha = .004; //multiplier for cap
    int last = 0; //last state of jopystick button
    int last2 =0;
    int capmode = 0; // 1 for in use, 0 for folded
    double beta = .05;
    double theta = 0;
    double d2beta = 0.4;

    //eytan's solution
    double height = 0, arm1 = 0, arm2 = 0;

    @Override
    public void runOpMode() {
        //region hardware map
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        capDrive = hardwareMap.get(DcMotor.class, "capDrive");
        capServo = hardwareMap.get(Servo.class,"capServo");
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
        capDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        capDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        //endregion

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        x = 0;
//      intakeServo.setPosition(1);
        waitForStart();

        while (opModeIsActive()) {

            intakeServo.setPosition(.45);

            //region drive code
            if (x == 0) {
                motorFL.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x)) * 1);
                motorBL.setPower((-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1);
//               motorFL.setPower(Math.max(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x)) * 1,((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x)) * d2beta));
//                motorBL.setPower(Math.max((-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1,(-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1*d2beta));
//                motorBR.setPower(Math.max(-((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1,-((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * d2beta));
//                motorFR.setPower(Math.max(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * 1,((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * d2beta));


            } else if (x == 1) {
                motorFL.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x)) * d2beta);
                motorBL.setPower((-(this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (-this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * d2beta);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (-this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * d2beta);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x)) * d2beta);
            }
            //endregion

            //region switch drive mode
            if (gamepad1.y && last2==1){

                if (x==1) x=0;
                else x = 1;
                telemetry.addData("x:", x);
                telemetry.update();
                robot.sleep(100);
                last2 = 0;


            } else {
                last2 = 1;
            }
            //endregion

            //region dump code
            if (gamepad1.b || gamepad2.a) {
                bucket.setPosition(.1);
                robot.sleep(1500);
                bucket.setPosition(.49);
                telemetry.addData("Position:", .5);
                telemetry.update();
            } else {
                bucket.setPosition(.5);
            }
            //endregion

            //region intake code
            int FLIntakePowerR = (int) gamepad1.right_trigger;
            boolean FLIntakePowerL = gamepad1.left_bumper;

            if (gamepad1.right_bumper) {
                intake.setPower(-1);
            } else if (FLIntakePowerL) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            //endregion

            //region lifter
            if (gamepad1.dpad_up) lifter.setPower(.8);
            else if (gamepad1.dpad_down) lifter.setPower(-.8);
           else if (gamepad2.dpad_up) lifter.setPower(.8);
            else if (gamepad2.dpad_down) lifter.setPower(-.8);
            else lifter.setPower(0);
            //endregion

            //region carousel mechanism
            if (gamepad1.dpad_left) {
                switch1Smoothed = ((1 * .005) + (switch1Prev * .995));
                switch1Prev = switch1Smoothed;
                telemetry.addData("speed", switch1Smoothed);
                telemetry.update();
                carousel.setPower(switch1Smoothed);
            } else if (gamepad1.dpad_right) {
                switch1Smoothed = ((1 * .005) + (switch1Prev * .995));
                switch1Prev = switch1Smoothed;
                carousel.setPower(-switch1Smoothed);
            } else {
                switch1Smoothed = 0;
                switch1Prev = 0;
                carousel.setPower(0);
            }
            //endregion

            //region capping mechanism

          /*  if (gamepad2.a) arm2 = .75;
            else if (gamepad2.y) arm2 = 1;
            else if (gamepad2.b) arm2 = .85;
            while (gamepad2.dpad_up){
                height++;
                telemetry.addData("height", height);
                telemetry.update();
                robot.sleep(10);
            }
            while (gamepad2.dpad_down){
                height--;
                telemetry.addData("height", height);
                telemetry.update();
                robot.sleep(10);
            }
            capDrive.setTargetPosition((int)((height*1425.1)/360));
            capDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            capDrive.setPower(.5);
            capServo.setPosition(arm2 - height/270);*/


            //endregion

            //region other capping mechanism
//            if (gamepad1.x) {
//                if (SWITCH == 0) {
//                    robot.newSleep(.5);
//                    SWITCH++;
//                } else if (SWITCH == 1) {
//                    robot.newSleep(.5);
//                    SWITCH--;
//                }
//            }
//            if (SWITCH == 0) liftyThingy.setPosition(1 - (gamepad1.right_trigger * .33));
//            if (SWITCH == 1) liftyThingy.setPosition(.66 + (gamepad1.right_trigger * .33));
            //endregion

            //region Real capping mechanism
            if (gamepad2.left_bumper) {
                theta = -beta;
            } else if ( gamepad2.left_trigger > .5) {
                theta = beta;
            } else {
                theta = 0;
            }
            if (capmode == 1){
                if (gamepad2.right_bumper) pos = pos + alpha;
                else if (gamepad2.right_trigger > .5) pos = pos - alpha;
                pos = Math.max(Math.min((double) 1, (pos)), (double) 0);

                capServo.setPosition(theta + ((double)robot.maps((long) (10000.0* pos), (long) 0, (long) 10000, (long) 660, (long) 260)) / (double) 1000);
                capDrive.setTargetPosition((int) robot.maps((long) (10000.0* pos), (long) 0, (long) 10000, (long) 0, (long) ticks));
                capDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                capDrive.setPower(.5);
            }
            else {
                capServo.setPosition((double) .95);
                capDrive.setTargetPosition((int) 0);
                capDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                capDrive.setPower(.5);
                pos = 0.0;


            }
            if (gamepad2.right_stick_button && last==1){

                if (capmode==1) capmode = 0;
                else capmode = 1;
                telemetry.addData("capmode:", capmode);
                telemetry.update();
               // robot.sleep(100);
                last = 0;

            } else {
                last = 1;
            }

            //endregion

        }
    }
}

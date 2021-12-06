package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class CompetitionDriving2021 extends LinearOpMode{

    private DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, lifter;
    //private Servo claw1, claw2,
    private Servo bucket;
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
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        //   claw1 = hardwareMap.get(Servo.class, "claw1");
        // claw2 = hardwareMap.get(Servo.class, "claw2");
        bucket = hardwareMap.get(Servo.class, "bucket");


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            x=0;




            if(x == 0){
                motorFL.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.75);
                motorBL.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.75);
                motorBR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.75);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.75);
            }

            else if(x == 1) {
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
            if (gamepad1.b){

                bucket.setPosition(.5);
                robot.sleep(1000);
                bucket.setPosition(1);
                bucketButton = !bucketButton;
            }


            if(gamepad1.right_bumper) {
                intakeFL.setPower(1);
            } else if(gamepad1.left_bumper){
                intakeFL.setPower(-1);
            } else {
                intakeFL.setPower(0);
            }
            if(gamepad1.dpad_up){
                lifter.setPower(.5);
            } else if (gamepad1.dpad_down){
                lifter.setPower(-.5);
            } else {
                lifter.setPower(0);
            }
            if (gamepad1.x) robot.setCarousel();
//hello there this is an easter egg

        }
    }
}

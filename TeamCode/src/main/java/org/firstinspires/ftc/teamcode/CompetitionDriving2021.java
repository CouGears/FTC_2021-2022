package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class CompetitionDriving2021 extends LinearOpMode{

    private DcMotor motorBR, motorBL, motorFL, motorFR;
    private Servo claw1, claw2;
    private boolean claw = false;

    int x = 0;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");



        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            x=0;




            if(x == 0){
                motorFL.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.75);
                motorBL.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.75);
                motorBR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.75);
                motorFR.setPower(((this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.75);
            }

            else if(x == 1) {
                motorFL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x)) * .25);
                motorBL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x)) * .25);
                motorBR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x)) * .25);
                motorFR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x)) * .25);
            }
            if(gamepad1.a){
                if (claw == false){
                    claw1.setPosition(.5);
                    claw2.setPosition(.5);
                    claw = !claw;
                }
                else {
                    claw1.setPosition(1);
                    claw2.setPosition(1);
                    claw = !claw;
                }
            }
        }
    }
}

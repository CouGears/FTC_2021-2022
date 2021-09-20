package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TestCode extends LinearOpMode{

    private DcMotor motorFL, motorFR;





    int x = 0;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorBL");



        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0){
                motorFR.setPower(gamepad1.right_trigger);

            } else if (gamepad1.left_trigger > 0){
                motorFR.setPower(gamepad1.left_trigger);
            } else{
                motorFR.setPower(0);
            }

        }
    }
}

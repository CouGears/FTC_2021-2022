package javaold.CompOne2021;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutonMethods;

@TeleOp

public class fourmotortestcode extends LinearOpMode {

    private DcMotor  intake, motorFR, motorBR, stucky;
    private boolean claw = false, bucketButton = false;
    private AutonMethods robot = new AutonMethods();
    int x = 0;


    @Override
    public void runOpMode() {
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stucky = hardwareMap.get(DcMotor.class, "stucky");


        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stucky.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        stucky.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
    if(gamepad1.a)
    {
        motorBR.setPower(1);
    }
   else if(gamepad1.b)
    {
        motorBR.setPower(-1);
    }
    else {
        motorBR.setPower(0);
    }
        if(gamepad1.x)
        {
            motorFR.setPower(1);
        }
       else if(gamepad1.y)
        {
            motorFR.setPower(-1);
        }
        else {
            motorFR.setPower(0);
        }
        if(gamepad1.dpad_up)
        {
            intake.setPower(1);
        }
      else if(gamepad1.dpad_down)
        {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }
        if(gamepad1.dpad_right)
        {
            stucky.setPower(1);
        }
       else if(gamepad1.dpad_left)
        {
            stucky.setPower(-1);
        }
        else {
            stucky.setPower(0);
        }

    }}}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import android.graphics.Color;

@TeleOp

public class CompetitionDriving extends LinearOpMode{
    private boolean serv = false, shoot = false, shooterServoToggle = false, lift = false, armToggle = false, armPos = true;
    private DcMotor motorBR, motorBL, motorFL, motorFR, intakeFL, shooter, arm, scissorMotor;
    private Servo shooterServo, armServo, frontScissor, armRaise, armBlock;




    int x = 0;


    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        intakeFL = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        scissorMotor = hardwareMap.get(DcMotor.class, "scissorMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        armBlock = hardwareMap.get(Servo.class, "armBlock");


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        scissorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setTargetPosition(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armBlock.setPosition(0.7);
        waitForStart();

        while (opModeIsActive()) {
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AutonMethods competition = new AutonMethods();
            /*(Color.RGBToHSV((int) (sensorColor.red() * 255),
                (int) (sensorColor.green() * 255),
                (int) (sensorColor.blue() * 255),
                hsvValues);

            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("In", in);
            telemetry.update();
            */
            if(gamepad1.dpad_left){
                x = 0;
            }

            else if(gamepad1.dpad_right){
                x = 1;
            }

            if(x == 0){
                motorFL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.75);
                motorBL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.75);
                motorBR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.75);
                motorFR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.75);
            }

            else if(x == 1){
                motorFL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.25);
                motorBL.setPower(-((this.gamepad1.left_stick_y) + (-this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.25);
                motorBR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (-this.gamepad1.left_stick_x))*.25);
                motorFR.setPower(((this.gamepad1.left_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_x))*.25);
            }
            if(gamepad1.right_bumper) {
                shooter.setPower(-.65);
            }else{
                shooter.setPower(0);
            }
            if(gamepad1.right_trigger > 0) {
                if (lift == true) {
                    scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    scissorMotor.setTargetPosition(0);
                    scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    scissorMotor.setPower(1);
                    telemetry.addData("Status", "y");
                    telemetry.update();
                    competition.sleep(500);
                    armBlock.setPosition(0.7);
                    lift = !lift;
                }
                intakeFL.setPower(gamepad1.right_trigger);
            } else if(gamepad1.left_bumper){
                intakeFL.setPower(-1);
            } else {
                intakeFL.setPower(0);
            }

            if(gamepad1.dpad_up && lift == false) {

                competition.sleep(150);
                scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                scissorMotor.setTargetPosition(-3150);
                scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                scissorMotor.setPower(1);
                telemetry.addData("Status", "x");
                telemetry.update();
                competition.sleep(500);
                armBlock.setPosition(0.5);
                lift = !lift;
            }
            if(gamepad1.dpad_down && lift == true){
                scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                scissorMotor.setTargetPosition(0);
                scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                scissorMotor.setPower(1);
                telemetry.addData("Status:", "true");
                telemetry.update();
                competition.sleep(500);
                armBlock.setPosition(0.7);
                lift = !lift;
            }


            /*if(gamepad2.a){
            shooter.setPower(0);
            intakeFL.setPower(0);
            }*/
            if(gamepad1.b){
                serv = !serv;
                try {
                    Thread.sleep(250);
                }
                catch (InterruptedException e) {
                }
            }
            if (serv == false){
                armServo.setPosition(.87);
            }else {
                armServo.setPosition(0.65); }
            // if(gamepad2.dpad_down){
            //     if (shooterServoToggle == true) { shooterServo.setPosition(0); competition.sleep(500); shooterServoToggle = !shooterServoToggle; }
            //     else if (shooterServoToggle == false) {  shooterServo.setPosition(.2); competition.sleep(500);  shooterServoToggle = !shooterServoToggle;}
            // }
            shooterServo.setPosition(gamepad1.left_trigger*.2);

            /*if (gamepad1.x){

                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (arm.getCurrentPosition() < 1750) {
                    arm.setPower(1);
                }
                arm.setPower(0);
                armServo.setPosition(.7);
                competition.sleep(300);
                armServo.setPosition(1);
            }*/
            if (gamepad1.y) {

                arm.setPower(0.5);
            }
            else if (gamepad1.a) {

                arm.setPower(-0.5);
            }
            else arm.setPower(0);

            if (gamepad1.x) {
                //   if (armPos == true){
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (arm.getCurrentPosition() < 10){
                    arm.setPower(1);
                }

                // armPos = !armPos;
                // }
            /*else if (armPos == false){
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                armPos = !armPos;
            }*/
            }

        }
    }
}

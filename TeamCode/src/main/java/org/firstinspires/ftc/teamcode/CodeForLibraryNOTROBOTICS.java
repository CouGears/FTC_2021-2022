package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import javaold.SensorSet.LEDMethods;

@TeleOp
//Hi Oran
public class CodeForLibraryNOTROBOTICS extends LinearOpMode {

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
        LEDMethods LED = new LEDMethods();
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

          while (gamepad1.dpad_up){
              motorBR.setPower(1);
              motorFR.setPower(-1);
          }
          while(gamepad1.dpad_down){
              motorBR.setPower(-1);
              motorFR.setPower(1);
          }
        }
    }
}

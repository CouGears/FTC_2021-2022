package javaold.Odometry;

//import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import com.qualcomm.robotcore.hardware.Servo;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.*;

        import com.qualcomm.robotcore.util.ElapsedTime;

public class OdometryMethods {

    //Constructor
    public OdometryMethods() {

    }

    //Declare and initial variables

    double rev2 = 2048;
    double inch2 = rev2 / (2 * 3.14);
    double feet2 = inch2 * 12;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, FODO, SODO;
    public static LED red, green;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;
    double locationx;
    double locationy;
    int Fpos = FODO.getCurrentPosition();
    int Spos = SODO.getCurrentPosition();
    public int counter = 0;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        SODO = map.get(DcMotor.class, "SODO");
        FODO = map.get(DcMotor.class, "FODO");
        red = map.get(LED.class, "red");
        green = map.get(LED.class, "green");

        // armServo = map.get(Servo.class, "armServo");
        // shooterServo = map.get(Servo.class, "shooterServo");
        //   note - this is according to front orientation - front is in the front and back is in the back
        // also these should be configured accordingly
        

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FODO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SODO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FODO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SODO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FODO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SODO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        FODO.setDirection(DcMotorSimple.Direction.FORWARD);
        SODO.setDirection(DcMotorSimple.Direction.FORWARD);


        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }


    public void getLocation() {
        tele.addData("Updating Data", .5);
        tele.update();
        locationy = (Fpos / feet2);
        locationx = (Spos / feet2);
    }

    public void kill() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }


    public void setRed()
    {
        green.enable(false);
        green.enableLight(false);
        red.enable(true);
        red.enableLight(true);
    }
    public void setGreen()
    {
        red.enable(false);
        red.enableLight(false);
        green.enable(true);
        green.enableLight(true);
    }
    public void setAmber()
    {
        red.enable(true);
        red.enableLight(true);
        green.enable(true);
        green.enableLight(true);
    }
    public void movefb(double forwards) {
    getLocation();
        if (forwards > 0) {
            while (motorFR.isBusy() || motorFL.isBusy()) {
                if (runtime.seconds() > 3)
                {
                    setRed();
                    break;//do i need to runtime reset?
                }
            }
            while (forwards - locationy >= 1) {
                setAmber();
                getLocation();//do i need to runtime reset?
                motorFL.setPower(.5);
                motorBL.setPower(.5);
                motorBR.setPower(.5);
                motorFR.setPower(.5);
                //drive until robot.locationy>movefb
                tele.addData("move forwards", .5);
                tele.update();
            }
            while (forwards - locationy < 1) {
                setAmber();
                getLocation();//do i need to runtime reset?
                motorFL.setPower(.1);//do i need to runtime reset?
                motorBL.setPower(.1);//do i need to runtime reset?
                motorBR.setPower(.1);
                motorFR.setPower(.1);
                //drive until robot.locationy>movefb
                tele.addData("move forwards", .5);
                tele.update();//do i need to runtime reset?
            }
        } else if (forwards < 0) {
            while (motorFR.isBusy() || motorFL.isBusy()) {
                if (runtime.seconds() > 3) break;
            }
            while (locationy > forwards) {
                getLocation();
                motorFL.setPower(-.5);
                motorBL.setPower(-.5);
                motorBR.setPower(-.5);
                motorFR.setPower(-.5);
                //drive until robot.locationy<movefb
                tele.addData("move backwqards", .5);
                tele.update();
            }
        } else
        {
            setGreen();
            kill();
        }
    }

    public void moverl(double sideways) {
        getLocation();
        if (sideways > 0) {
            if (locationx < sideways) {
                while (motorFR.isBusy() || motorFL.isBusy()) {
                    if (runtime.seconds() > 3) break;
                }
                getLocation();
                motorFL.setPower(.5);
                motorBL.setPower(-.5);
                motorBR.setPower(.5);
                motorFR.setPower(-.5);
                //drive until robot.locationx>moverl
                tele.addData("move right", .5);
                tele.update();
            }
        } else if (sideways < 0) {
            if (locationx > sideways) {
                while (motorFR.isBusy() || motorFL.isBusy()) {
                    if (runtime.seconds() > 3) break;
                }
                getLocation();
                motorFL.setPower(-.5);
                motorBL.setPower(.5);
                motorBR.setPower(-.5);
                motorFR.setPower(.5);
                //drive until robot.locationx<moverl
                tele.addData("move left", .5);
                tele.update();
            }
        } else kill();
    }
}


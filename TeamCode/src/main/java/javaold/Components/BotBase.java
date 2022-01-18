package javaold.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BotBase {

    //Declare and initial variables
    public DcMotor rearRightDrive, rearLeftDrive, frontLeftDrive, frontRightDrive, intakeFL, shooter, arm, scissorMotor;
    private static Servo shooterServo, armServo, marker, frontScissor, backScissor;
    public static Servo armBlock;
	public GyroSensor gyro;
    public static DistanceSensor topSensor, bottomSensor;
    private ElapsedTime runtime = new ElapsedTime();
    Telemetry tele;

    private double speed;
    private boolean clampDown = false;
    public int counter = 0;


    private Boolean hasOdometry = false;

    public Odometry odometry  = null;

    public int verticalEncoderRadiusFromCenter = 5;
    public int HorizontalEncoderRadiusFromCenter = 5;

    Orientation angles;

    //Initialization

    public BotBase(HardwareMap hardwareMap, Telemetry tele) {
        /*frontLeftDrive = hardwareMap.get(DcMotor.class, "motorFL");
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeftDrive = hardwareMap.get(DcMotor.class, "motorBL");
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);


        rearRightDrive = hardwareMap.get(DcMotor.class, "motorBR");
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);


        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeFL = hardwareMap.get(DcMotor.class, "intake");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottomSensor");
        topSensor = hardwareMap.get(DistanceSensor.class, "topSensor");

        armServo = hardwareMap.get(Servo.class, "armServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        //note - this is according to front orientation - front is in the front and back is in the back
        //also these should be configured accordingly
        scissorMotor = hardwareMap.get(DcMotor.class, "scissorMotor");
        armBlock = hardwareMap.get(Servo.class, "armBlock");

        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottomSensor");
        topSensor = hardwareMap.get(DistanceSensor.class, "topSensor");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        intakeFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeFL.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeftDrive.setTargetPosition(0);
        rearLeftDrive.setTargetPosition(0);
        frontRightDrive.setTargetPosition(0);
        rearRightDrive.setTargetPosition(0);

        scissorMotor.setTargetPosition(0);
		
		gyro = hardwareMap.gyroSensor.get("gyro");
		

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!gyro.isCalibrating())
        {
        }
        gyro.resetZAxisIntegrator();
        odometry = new Odometry(frontLeftDrive,rearRightDrive,gyro,verticalEncoderRadiusFromCenter,HorizontalEncoderRadiusFromCenter);*/

    }
    public void stop() {
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        rearLeftDrive.setPower(0.0);
        rearRightDrive.setPower(0.0);
    }
    public Boolean hasOdometry() { return hasOdometry; }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public DcMotor getRearRightDrive() {
        return rearRightDrive;
    }

    public DcMotor getRearLeftDrive() {
        return rearLeftDrive;
    }
    public void updateComponents() {

        if (hasOdometry()) {
            odometry.globalCoordinatePositionUpdate();
        }
    }

}

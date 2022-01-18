package javaold.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class odometrytest extends LinearOpMode {

    private DcMotor motorBL, motorFR, motorBR, motorFL, carousel, FODO, SODO;
    private OdometryMethods odometry = new OdometryMethods();
    int x = 0;


    @Override
    public void runOpMode() {
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        FODO = hardwareMap.get(DcMotor.class, "FODO");
        SODO = hardwareMap.get(DcMotor.class, "SODO");


        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FODO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SODO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        FODO.setDirection(DcMotorSimple.Direction.FORWARD);
        SODO.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Activated");
            telemetry.update();
            switch (odometry.counter) {
                case 0:
                    odometry.movefb(1);
                    odometry.counter++;
                    break;
            }
        }
    }
}
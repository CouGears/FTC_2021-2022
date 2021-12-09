package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.BotBase;


@Autonomous(name="Autonomous Base Class", group="none")
@Disabled
public class AutonomousOdometryBase extends OpMode {

    public BotBase botBase = null;

    protected ElapsedTime runtime = new ElapsedTime();
    static final double DRIVE_SPEED                     = 0.8;

    /**
     * Inits the the automomous mode
     * 1) imports the hardware
     * 2) Initialize the hardware
     */
    protected void initAutonomous() {
        botBase = new BotBase(hardwareMap, telemetry);
    }

    /**
     * Always insert this function inside a control loop.
     */
    protected void autonomousIdleTasks() {
        botBase.updateComponents();
        telemetry.addData("RawZ", botBase.odometry.getOrientation());
        telemetry.update();

    }
    protected void moveDiagonally(double angleInDegrees, double distance, double power)
    {

        angleInDegrees = angleInDegrees % 360;
        if (angleInDegrees < 0) {
            angleInDegrees = 360 + angleInDegrees;
        }

        if (angleInDegrees == 0.0) {
            return;
        }
        if (angleInDegrees == 90.0) {
            return;
        }
        if (angleInDegrees == 180.0) {
            return;
        }
        if (angleInDegrees == 270.0) {
            return;
        }

        double angleInRadians = angleInDegrees * Math.PI / 180.0;
        double targetY = Math.sin(angleInRadians);
        double targetX = Math.cos(angleInRadians);

        MoveAngle(targetX, targetY, power);
        return;
    }

    /*
     *
     * Need Odometer
     */
    private void MoveAngle(double targetX, double targetY, double power) {
        if (!botBase.hasOdometry()) {
            return;
        }
            double x = botBase.odometry.getCurrentXPos();
            double y = botBase.odometry.getCurrentYPos();

            double MovementAngle = Math.atan2(targetX, targetY);

            //recalculate the angle
            powerPropulsionAtAngle(MovementAngle, power * 1.5);

        return;
    }
    private void MoveTo(double targetX, double targetY, double power) {
        if (!botBase.hasOdometry()) {
            return;
        }

        double x = botBase.odometry.getCurrentXPos();
        double y = botBase.odometry.getCurrentYPos();
        int botAngle = botBase.odometry.getOrientationLocal();

        double Angle = Math.atan2(targetX-x, targetY-y);
        double MovementAngle = Angle - botAngle -45;
        //recalculate the angle
        powerPropulsionAtAngle(MovementAngle, power * 1.5);

        return;
    }
    protected void powerPropulsionAtAngle(double angleInRadians, double power) {
        if (power == 0) {
            power = DRIVE_SPEED;
        }

        double temp = angleInRadians + Math.PI/2.0;

        double front_left = power * Math.cos(temp);
        double front_right = power * Math.sin(temp);
        double rear_left = power * Math.sin(temp);
        double rear_right = power * Math.cos(temp);

        // normalize the wheel speed so we don't exceed 1
        double max = Math.abs(front_left);
        if (Math.abs(front_right)>max) {
            max = Math.abs(front_right);
        }
        if (Math.abs(rear_left)>max){
            max = Math.abs(rear_left);
        }
        if (Math.abs(rear_right)>max) {
            max = Math.abs(rear_right);
        }
        if ( max > 1.0 ) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }
        botBase.getFrontLeftDrive().setPower(front_left);
        botBase.getFrontRightDrive().setPower(front_right);
        botBase.getRearLeftDrive().setPower(rear_left);
        botBase.getRearRightDrive().setPower(rear_right);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}

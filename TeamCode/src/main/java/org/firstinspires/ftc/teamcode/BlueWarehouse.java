
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous

public class BlueWarehouse extends OpMode {

    //TensorFlowVision vision = new TensorFlowVision();
//   double rev = 383.6; //435 rpm motor
    double rev = 537.7; //312 rpm motor
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12 + (10 * inch);

    private ElapsedTime runtime = new ElapsedTime();
    AutonMethods robot = new AutonMethods();
    HardwareMap map;
    Telemetry tele;

    @Override
    public void init() {
        // Tell the driver that initialization is complete.
        // bot.initAutonomous();
        // vision.init();

        robot.init(hardwareMap, telemetry, false);


        //int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //bot.autonomousIdleTasks();
        //vision.check();
//        int tie = 0;
        switch (robot.counter) {
            case 0:
                robot.drive(-1.25*feet, 0*feet, .5);
                robot.counter++;
                break;
            case 1:
                robot.distanceSetBack();

                if(robot.dist <= 15)
                {
                    robot.BlockPosition=2;
                }
                robot.counter=2;
                break;
            case 2:
                robot.drive(0*feet, -8*inch, .5);
                robot.counter++;
                break;
            case 3:
                robot.distanceSetBack();

                if(robot.dist <= 15)
                {
                    robot.BlockPosition=3;
                }
                robot.counter++;
                break;
            case 4:
                robot.drive(0*feet, -18*inch, .5);
                robot.counter++;
                break;
            case 5:
            robot.blockPos();
            robot.counter++;
                break;
            case 6:
                robot.turn(90);
                robot.counter++;
                break;
            case 7:
                robot.drive(0*inch, -33*inch, .5);
                robot.counter++;
                break;
            case 8:
                robot.drive(4*feet, 0 ,.5);
                robot.counter++;
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

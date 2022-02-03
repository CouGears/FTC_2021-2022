
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
                robot.drive(0, -.42*feet,.5);
                robot.counter++;
                break;
            case 1:
                robot.drive(-1.25*feet, 0*feet, .5);
                telemetry.addData("Move to Scan",1);
                telemetry.update();
                robot.counter++;
                break;
            case 2:
                robot.distanceSetBack();

                if(robot.distBack <= 15)
                {
                    robot.BlockPosition=2;
                    telemetry.addData("Scanned - Block in POS",2);
                    telemetry.update();

                }
                robot.counter++;
                break;
            case 3:
                robot.drive(0.01*feet, -.66*feet, .5);
                telemetry.addData("Move to Scan",2);
                telemetry.update();
                robot.counter++;
                break;
            case 4:
                robot.distanceSetBack();

                if(robot.distBack <= 10)
                {
                    robot.BlockPosition=3;
                    telemetry.addData("Scanned - Block in POS",3);
                    telemetry.update();

                }
                robot.counter++;
                break;
            case 5:
                robot.drive(0.01*feet, -1.5*feet, .5);
                telemetry.addData("Move to Dump",0);
                telemetry.addLine();
                telemetry.addData("dumping in position" , robot.BlockPosition);
                robot.newSleep(2);
                robot.counter++;
                break;
            case 6:
            robot.blockPos();
            robot.newSleep(2);
            robot.counter++;
                break;
            case 7:
                robot.dump();
                robot.counter++;
                break;
            case 8:
                robot.turn(90);
                robot.counter++;
                break;
            case 9:
                robot.drive(0*inch, -36*inch, .5);
                robot.counter++;
                break;
            case 10:
                robot.drive(4.5*feet, 0 ,.5);
                robot.counter++;
                break;
            case 11:
                robot.autonLower();
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

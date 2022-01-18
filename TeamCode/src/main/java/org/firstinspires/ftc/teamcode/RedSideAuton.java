package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous

public class RedSideAuton extends OpMode {

    //TensorFlowVision vision = new TensorFlowVision();
//   double rev = 383.6; //435 rpm motor
    double rev = 537.7; //312 rpm motor
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12 + (10*inch);

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
                robot.setCarousel();
                robot.setIntake(1);
                telemetry.addData("spot: carousel", 1);
                telemetry.update();
//                robot.turn(180);
//                    robot.drive(0, -5*feet ,.5); // I use this one to test the robot
                robot.counter++;
                break;
            case 1:
                robot.turn(25);
                telemetry.addData("spot: turn", 1);
                    telemetry.update();
                robot.counter++;
                break;
            case 2:
                robot.drive(-.3*feet, 3.2 * feet,.5); //drives to scan point
                robot.sleep(2000);
                robot.counter++;
                break;

            case 3:
                robot.turn(180);
                robot.counter++;
                break;
           case 4:
                robot.drive(-1.9 * feet, -1.5 * feet,.5); //drive to drop point
                robot.counter++;
                break;
           /* case 5:
               if (robot.distance() == 1) {
                    robot.lift(400);//Top of the tower
                    robot.lift();
                } else if (robot.distance() == 2) {
                    robot.lift(200);//Middle
                    robot.lift();
                } else if (robot.distance() == 3) {
                    robot.lift(0);//Bottom of the tower
                    robot.lift();
                }
                telemetry.addData("spot", robot.distance());
                telemetry.update();

                robot.counter++;
                break;
            case 6:
                robot.drive(.5 * feet, 2 * feet,.5);
              //  robot.drive(8 * feet, 0, .5);
                robot.drive(0, -.5 * feet,.5);
                //robot.drive(-5 * feet, 0, .5);
               // robot.turn(90);
               // robot.drive(-2 * feet, 0, .5);
                robot.counter++;
                break;*/
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

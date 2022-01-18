package javaold.CompOne2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.AutonMethods;

@Autonomous

public class BlueSideComp1 extends OpMode {
    double rev = 537.6;
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    AutonMethods robot = new AutonMethods();
    int diamond = 0;

    public void init() {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "This worked");
        telemetry.update();


    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        switch (robot.counter) {
            case 0:
                robot.intakeServo.setPosition(0);
                robot.drive(15*feet, 0, 1);
                robot.counter++;
                break;
          /*  case 1:
                robot.setCarousel();
                robot.counter++;
                break;
            case 2:
                robot.drive(6*feet, 0, 1); //drive to drop point
                robot.counter++;
                break;
*/
        }
    }
}

package javaold.SensorSet;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LEDMethods {

    //Constructor
    public LEDMethods() {

    }

    private final ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;
    public static LED red, green, red2, green2;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        // location[0] = 0;
        //location[1] = 0;

        red = map.get(LED.class, "red");
        green = map.get(LED.class, "green");
        red2 = map.get(LED.class, "red2");
        green2 = map.get(LED.class, "green2");

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

        // tele.addData(">", "Gyro Calibrating. Do Not Move!");
        // tele.update();
    }
public void LED1(int state)
    {
        if(state==1)setRed();
        if(state==2)setAmber();
        if(state==3)setGreen();
    }
    public void LED2(int state)
    {
        if(state==1)setRed2();
        if(state==2)setAmber2();
        if(state==3)setGreen2();
    }
    public void setRed2() {
        green2.enable(false);
        red2.enable(true);
    }

    public void setGreen2() {
        red2.enable(false);
        green2.enable(true);
    }

    public void setAmber2() {
        red2.enable(true);
        green2.enable(true);
    }

    public void setRed() {
        green.enable(false);
        red.enable(true);
    }

    public void setGreen() {
        red.enable(false);
        green.enable(true);
    }

    public void setAmber() {
        red.enable(true);
        green.enable(true);
    }
}
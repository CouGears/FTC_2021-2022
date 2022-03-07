package javaold.SensorSet;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MagnetMethods {

    //Constructor
    public MagnetMethods() {

    }
    TouchSensor magnet;
    public int magnetSense;
    HardwareMap map;
    Telemetry tele;
    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton)
    {
        magnet = map.get(TouchSensor.class, "magnet");
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
    }



        // If the Magnetic Limit Swtch is pressed, stop the motor
        public int LimitPressed()
        {
            if (magnet.isPressed()) {
                magnetSense = 1;
                return magnetSense;
            }
            else
            {
                magnetSense = 0;
                return magnetSense;
            }
        }
    }
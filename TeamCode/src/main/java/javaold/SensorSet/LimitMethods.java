package javaold.SensorSet;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class LimitMethods {

    //Constructor
    public LimitMethods() {

    }
    TouchSensor limit;
    public int limitPressed;
    HardwareMap map;
    Telemetry tele;
    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton)
    {
        limit = map.get(TouchSensor.class, "Limit");
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
    }



        // If the Magnetic Limit Swtch is pressed, stop the motor
        public int LimitPressed()
        {
            if (limit.isPressed()) {
                limitPressed = 1;
                return limitPressed;
            }
            else
            {
                limitPressed = 0;
                return limitPressed;
            }
        }
    }
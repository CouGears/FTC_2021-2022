package javaold.SensorSet;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColorSensorMethods {
    public static com.qualcomm.robotcore.hardware.ColorSensor color;

    //Constructor
    public ColorSensorMethods() {

    }

    private final ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry telemetry;
    public int colorMatch;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        color = map.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "Color");
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
    }

    public int sense(int red, int green, int blue, int marginRed, int marginGreen, int marginBlue) {
        if (color.red() <= red + marginRed && color.red() >= red - marginRed && color.green() <= green + marginGreen && color.green() >= green - marginGreen && color.blue() <= blue + marginBlue && color.blue() >= blue - marginBlue) {
            colorMatch = 1;
            return colorMatch;
        }
        else
        {
            colorMatch=0;
            return colorMatch;
        }
    }
}
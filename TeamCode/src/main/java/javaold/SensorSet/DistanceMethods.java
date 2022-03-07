package javaold.SensorSet;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceMethods {

    //Constructor
    public DistanceMethods() {

    }
public double frontDistance, backDistance;
    public static DistanceSensor distanceSensor, distanceSensorBack;
    private final ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;
    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        distanceSensor = map.get(DistanceSensor.class, "distanceSensor");
        distanceSensorBack = map.get(DistanceSensor.class, "distanceSensorBack");
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
    }
    public double getFront() {
        frontDistance = distanceSensor.getDistance(DistanceUnit.CM);
        return frontDistance;
    }
    public double getBack() {
        backDistance = distanceSensorBack.getDistance(DistanceUnit.CM);
        return backDistance;
    }
}
package javaold.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

public class Odometry {

    static final double COUNTS_PER_INCH = 6.157521601;

    /**
     * HARDWARE
     */
    private final DcMotor verticalEncoder;
    private final DcMotor horizontalEncoder;
	private final GyroSensor gyro;

    /**
     * VARIABLES
     */
    double verticalEncoderRadiusOffset;
    double horizontalEncoderRadiusOffset;

    // Position variables used for storage and calculations
    double verticalEncoderWheelPosition = 0;
    double horizontalEncoderWheelPosition = 0;


    private int changeInRobotOrientation = 0;
    private int previousOrientation = 0;
    private int robotOrientationRadians = 0;

    double distanceToTarget = 0;

    private double robotGlobalXCoordinatePosition = 0;
    private double robotGlobalYCoordinatePosition = 0;

    private double previousVerticalEncoderWheelPosition = 0;
    private double previousHorizontalEncoderWheelPosition = 0;


    /**
     * CONSTANTS FROM CONFIGURATION FILE
     */


    public Odometry(DcMotor verticalEncoder,
                     DcMotor horizontalEncoder, GyroSensor gyro, double verticalEncoderRadiusOffset, double horizontalEncoderRadiusOffset) {
        
        this.verticalEncoder = verticalEncoder;
        this.horizontalEncoder = horizontalEncoder;
		this.gyro = gyro;
		this.verticalEncoderRadiusOffset = verticalEncoderRadiusOffset;
        this.horizontalEncoderRadiusOffset = horizontalEncoderRadiusOffset;
    }


    public void globalCoordinatePositionUpdate() {
   /*     robotOrientationRadians = getRobotOrientationRadians();*/
        changeInRobotOrientation = robotOrientationRadians - previousOrientation;

        //This might need changes
        double OffsetX = (Math.cos(robotOrientationRadians)*horizontalEncoderRadiusOffset) - (Math.cos(previousOrientation)*horizontalEncoderRadiusOffset);
        double OffsetY = (Math.sin(robotOrientationRadians)*verticalEncoderRadiusOffset) - (Math.sin(previousOrientation)*verticalEncoderRadiusOffset);

        horizontalEncoderWheelPosition = horizontalEncoder.getCurrentPosition();
        double rawHorizontalChange = horizontalEncoderWheelPosition - previousHorizontalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*0);

        verticalEncoderWheelPosition= verticalEncoder.getCurrentPosition();
        double rawVerticalChange = verticalEncoderWheelPosition - previousVerticalEncoderWheelPosition;
        double verticalChange = rawVerticalChange - (changeInRobotOrientation*0);

        distanceToTarget = Math.hypot(horizontalChange,verticalChange);

        double theta = Math.atan2(verticalChange,horizontalChange);

        // Calculate and update the actual position values
        robotGlobalXCoordinatePosition += distanceToTarget * Math.cos(theta+robotOrientationRadians) *COUNTS_PER_INCH;
        robotGlobalYCoordinatePosition += distanceToTarget * Math.sin(theta+robotOrientationRadians) *COUNTS_PER_INCH;

        robotGlobalXCoordinatePosition +=OffsetX;
        robotGlobalYCoordinatePosition +=OffsetY;

        // Update state variables
        previousVerticalEncoderWheelPosition = verticalEncoderWheelPosition;
        previousHorizontalEncoderWheelPosition = horizontalEncoderWheelPosition;
        previousOrientation = robotOrientationRadians;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double getCurrentXPos(){
        return robotGlobalXCoordinatePosition;
    }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double getCurrentYPos() {
        return robotGlobalYCoordinatePosition;
    }
    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public int getOrientation() {
        return robotOrientationRadians;
    }

    public int getOrientationLocal() {
        return (int) (robotOrientationRadians-(Math.PI/2));
    }
  /*  private int getRobotOrientationRadians(){
        return gyro.rawZ();*/
    }



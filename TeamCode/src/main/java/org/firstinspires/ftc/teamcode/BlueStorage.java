/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous

public class BlueStorage extends OpMode {

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
        robot.setServo();
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
                robot.drive(0 * feet, .5*feet, .5);//move to carousel
//                robot.turn(180);
//                    robot.drive(0, -5*feet ,.5); // I use this one to test the robot
                robot.sleep(1500);
                robot.counter++;
                break;
            case 1:
                robot.drive(-1.3 * feet, 0*feet, .5);//move to carousel
//                robot.turn(180);
//                    robot.drive(0, -5*feet ,.5); // I use this one to test the robot
                robot.sleep(1500);
                robot.counter++;
                break;
            case 2:
                robot.setCarousel(-.7);//move carousel
                robot.setIntake(1);
                telemetry.addData("spot: carousel", 1);
                telemetry.update();
                robot.counter++;
                break;
            case 3:
                robot.drive(0 * feet, 1.5 * feet, .5); //drives right to scan point
                robot.counter++;
                break;
            case 4:
                robot.drive(-.33 * feet, 0.01 * feet, .5); //drives back to scan point
                robot.newSleep(.01);
                robot.counter++;
                break;
            case 5:
                robot.drive(0 * feet, 1.3 * feet, .5); //drives right to scan point
                //needs three to be in right position and avoid carousel
                robot.counter++;
                break;
            case 6:
               robot.newSleep(1);
                robot.distanceSet();//grab distance of block and know how to move arm this is stored and stay same but right leg of robot needs to line up with barcode for this system to work(sorry thatll be change case 4)
                telemetry.addData("spot", robot.distanceSet());
                telemetry.update();
                robot.newSleep(1);
                robot.counter++;
                break;
            case 7:
                robot.drive(0.01 * feet, .8 * feet, .5); //drive to drop point move over
                robot.counter++;
                break;
            case 8:
                robot.turn(180);//turn to drop
                robot.newSleep(2);//wait for completion so we dont drive on angle
                robot.counter++;
                break;
            case 9:
                robot.drive(0 * feet, -0.4 * feet, .5); //drive to drop point

                //robot.newSleep(.5);
                robot.counter++;
                break;
            case 10:
                robot.drive(-2.4 * feet, 0 * feet, .5); //drive to drop point
                robot.counter++;
                break;
            case 11:
                //robot.newSleep(2);
                if (robot.distance() == 1500) {//this is all preconfigured and stays the same
                    robot.drive(.35*feet,0,1);
                    robot.lift(1400);//Bottom of the tower
                    robot.setRed();
                    robot.setRed2();
                    telemetry.addData("spot - bottom", robot.distance());
                    telemetry.update();
                } else if (robot.distance() == 2000) {
                    robot.drive(.2*feet,0,1);
                    robot.lift(2000);//Middle
                    robot.setAmber();
                    robot.setAmber2();
                    telemetry.addData("spot - middle", robot.distance());
                    telemetry.update();
                } else if (robot.distance() == 3300) {
                    robot.drive(.2*feet,0,1);
                    robot.lift(3300);
                    robot.setGreen();
                    robot.setGreen2();
                    telemetry.addData("spot - top", robot.distance());
                    telemetry.update();//Top of the tower
                }

                robot.newSleep(2);
                robot.counter++;
                break;
            case 12:
                robot.dump();//configured stays the same
               robot.newSleep(.5);
                if (robot.distance()==1500){
                    robot.drive(-.4*feet,0,1);
                } else if (robot.distance()==2000) {
                robot.drive(-.2*feet,0,1);
            }
                robot.counter++;
                break;
            case 13:
                robot.drive(2.8 * feet, 0 * feet, .5);//move to park
                robot.drive(0,1.35*feet,1);
                robot.counter++;
                break;
            case 14:
                //open servo to lower arm
//                robot.intakeServo.setPosition(.45);
                robot.counter++;
                break;
            case 15:
                if (robot.crap == 3300) {//this is all preconfigured and stays the same
                    robot.lift(-3300);//Top of the tower
                } else if (robot.crap == 2500) {
                    robot.lift(-2000);//Middle
                } else if (robot.crap == 2000) {
                    robot.lift(-1500);//Bottom of the tower
                }
                robot.counter++;
                break;

                /*    case 8:
                robot.drive(1*feet,0,.5);
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

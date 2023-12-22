/*
Copyright 2023 FIRST Tech Challenge Team 6556

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.IMU;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.firstinspires.ftc.robotcore.external.JavaUtil;
 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.hardware.dfrobot.HuskyLens;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
 
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 
 
 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import org.firstinspires.ftc.robotcore.internal.system.Deadline;
 
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.Position;
 import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
 import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
 
 import java.util.concurrent.TimeUnit;
 

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class TempestAutoRedLong extends LinearOpMode {
    private DcMotor LeftBack;
    private DcMotor RightBack;
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor arm;
    private Servo drone_servo;
    private CRServo extension_servo;
    private HuskyLens huskyLens;
    private Servo leftservo;
    private DcMotor liftmotor;
    private Servo rightservo;
    private Servo top_servo;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    static final double ENCODER_CLICKS = 537.7;    // REV 40:1  1120
     static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
     static final double WHEEL_CIRC = 3.78;     // For figuring circumference
     static final double COUNTS_PER_INCH = (ENCODER_CLICKS * DRIVE_GEAR_REDUCTION) /
             (WHEEL_CIRC * 3.1415);
     static final double DRIVE_SPEED = 1.0;
     static final double TURN_SPEED = 0.5;
     static final double speed = 0.5;
     
     double LensPosition = 0;
    
      private final int READ_PERIOD = 1;


    @Override
    public void runOpMode() {
        LeftBack = hardwareMap.get(DcMotor.class, "Back_left");
        RightBack = hardwareMap.get(DcMotor.class, "Back_right");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        LeftFront = hardwareMap.get(DcMotor.class, "Front_left");
        RightFront = hardwareMap.get(DcMotor.class, "Front_right");
        arm = hardwareMap.get(DcMotor.class, "arm");
        drone_servo = hardwareMap.get(Servo.class, "drone servo");
        extension_servo = hardwareMap.get(CRServo.class, "extension servo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        liftmotor = hardwareMap.get(DcMotor.class, "liftmotor");
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        top_servo = hardwareMap.get(Servo.class, "top servo");

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

         Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
         
         //wheel setup
          
         RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         
            
         RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         
         
         LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         
          LeftFront.setDirection(DcMotor.Direction.REVERSE);
          LeftBack.setDirection(DcMotor.Direction.REVERSE);


        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                telemetry.addData("Block position", blocks[i].x);
                
                 if ((blocks[i].x > 20) && (blocks[i].x < 110)) {
                 telemetry.addData("IT WORKS", "");
                 LensPosition = 1;
                  telemetry.addData("POSITION", LensPosition);
                  path1();
                 }
                 if ((blocks[i].x > 140) && (blocks[i].x < 240)) {
                 telemetry.addData("IT WORKS", "");
                 LensPosition = 2;
                  telemetry.addData("POSITION", LensPosition);
                    path2();
                 }
                 if ((blocks[i].x > 260) && (blocks[i].x < 340)) {
                 telemetry.addData("IT WORKS", "");
                 LensPosition = 3;
                  telemetry.addData("POSITION", LensPosition);
                    path3();
                 }
            }
             telemetry.addData("Done", LensPosition);
            telemetry.update();
        } // end of while opmode
        
        
        //operational methods
      
       
       
       
        
    }
      //operational methods
    public void path1() {
    //drivebot format: driveBot(Leftdistanceinfeet, rightdistanceinfeet, speed, runtime)
    driveBot(2.5, 2.5, 0.5, 3);
    left90();
     driveBot(0.3, 0.3, 0.5, 3);
     servo();
     sleep(750);
     driveBot(-0.5, -0.5, 0.5, 2);
     right90();
     driveBot(2.25, 2.25, 0.5, 3);
     left90();
      driveBot(8.5, 8.5, 0.7, 6);
    sleep(30000);
       }
       
    public void path2() {
     driveBot(2.5, 2.5, 0.5, 5);
      servo();
     sleep(750);
      driveBot(-0.5,-0.5, 0.3, 3);
      left90();
      driveBot(-1.5, -1.5, 0.5, 5);
      right90();
      driveBot(3.2, 3.2, 0.5, 3);
       left90();
      driveBot(9.75, 9.75, 0.7, 6);
    sleep(30000);
       }
       
    public void path3() {
     driveBot(2.5, 2.5, 0.5, 3);
    right90();
     servo();
     sleep(750);
     driveBot(-0.2, -0.2, 0.5, 2);
     left90();
     driveBot(2.25, 2.25, 0.5, 3);
     left90();
      driveBot(8.5, 8.5, 0.7, 6);
    sleep(30000);
       }
       
    public void servo() {
    leftservo.setPosition(1);
    }
       
       
    /**********************************************
      * BEGIN METHODS driveDistance, driveBot
      *********************************************/
     
     /**
     * Drive Distance Method
     * Calculate distance from CM to encoder counts
     */  
     public static double driveDistance(double distance)
     {
         double drive  = (ENCODER_CLICKS/ WHEEL_CIRC);
         int outputClicks= (int)Math.floor(drive * distance * 6/20);
         return outputClicks;
     }
     // END driveDistance() method
 
 
 public void driveBot(double distanceInInleft, double distanceInInright, double power, double timeoutS) 
     {
         telemetry.addData("status","encoder reset");
         telemetry.update();
         
         int rightTarget;
         int leftTarget;
 
         if(opModeIsActive()) 
         {
             
             telemetry.update();
             
             rightTarget = (int) driveDistance(distanceInInright * 12);
             leftTarget = (int) driveDistance(distanceInInleft * 12);
 
             RightFront.setTargetPosition(rightTarget);
             LeftFront.setTargetPosition(leftTarget);
             RightBack.setTargetPosition(rightTarget);
             LeftBack.setTargetPosition(leftTarget);
 
             RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             
             runtime.reset();
 
             RightFront.setPower(power);
             LeftFront.setPower(power);
             RightBack.setPower(power);
             LeftBack.setPower(power);
         
             while (opModeIsActive() &&
                 (runtime.seconds() < timeoutS) &&
                 (LeftFront.isBusy() && RightFront.isBusy() ))
                 {
                    telemetry.addData("POSITION", LensPosition);
                     telemetry.update();
                 }
             LeftFront.setPower(0);
             RightFront.setPower(0);
             LeftBack.setPower(0);
             RightBack.setPower(0);
             LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             
              RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }   
     }
     // END driveBot method
    
             
       
       
       //Turn left 90
       public void left90() {
           driveBot(-1.92,1.92,speed,3);
             telemetry.addData("status","Check position 2 for object" );
             telemetry.addData("status", LeftFront.getMode() );
             telemetry.addData("status","left motor,  %7d", LeftFront.getCurrentPosition() );
             telemetry.addData("status","right motor,  %7d", RightFront.getCurrentPosition() );
             telemetry.update();
             RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           
       }
       
       
       //Turn Right 90
       public void right90() {
           driveBot(1.92,-1.92,speed,3);
             telemetry.addData("status","Check position 2 for object" );
             telemetry.addData("status", LeftFront.getMode() );
             telemetry.addData("status","left motor,  %7d", LeftFront.getCurrentPosition() );
             telemetry.addData("status","right motor,  %7d", RightFront.getCurrentPosition() );
             
       }
       
       //Turn Right 180
       public void right180() {
           driveBot(3.9,-3.9,speed,5);
             telemetry.addData("status","Check position 2 for object" );
             telemetry.addData("status", LeftFront.getMode() );
             telemetry.addData("status","left motor,  %7d", LeftFront.getCurrentPosition() );
             telemetry.addData("status","right motor,  %7d", RightFront.getCurrentPosition() );
             
       }
       
       
}

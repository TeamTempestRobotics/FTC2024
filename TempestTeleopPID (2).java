package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TempestTeleopPID")
public class TempestTeleopPID extends LinearOpMode {

  private DcMotor arm;
  private Servo droneservo;
  private DcMotor liftmotor;
  private DcMotor Back_right;
  private DcMotor Front_right;
  private DcMotor Front_left;
  private DcMotor Back_left;
  private CRServo extensionservo;
  private Servo topservo;
  private Servo rightservo;
  private Servo leftservo;
  
  
//speed for all superstructure motors
    static final double ARMSPEED = 1;
    
    
    //Soft limits
     //Soft limits
    static final int ArmLimit = -1300;
//Integer calls for setpoint
                    int ArmTarget = 0;

  /**
   * Describe this function...
   */
  private void Arm() {
      double ArmInputraw = gamepad2.left_stick_y * 15;
      int ArmInput = (int) Math.floor(ArmInputraw);
      ArmTarget = ArmTarget + (ArmInput);
      
        arm.setPower(ARMSPEED);
        arm.setTargetPosition(ArmTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      //Arm control/limit
                if (ArmTarget < ArmLimit) {
                   ArmTarget = ArmLimit;
                } else if(ArmTarget > 0) {
                    ArmTarget = 0;
                } else {
                    ArmTarget = ArmTarget;
                }
  }

  /**
   * Describe this function...
   */
  private void drone_launch() {
    if (gamepad1.b) {
      droneservo.setPosition(0);
    } else {
      droneservo.setPosition(1);
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    arm = hardwareMap.get(DcMotor.class, "arm");
    droneservo = hardwareMap.get(Servo.class, "drone servo");
    liftmotor = hardwareMap.get(DcMotor.class, "liftmotor");
    Back_right = hardwareMap.get(DcMotor.class, "Back_right");
    Front_right = hardwareMap.get(DcMotor.class, "Front_right");
    Front_left = hardwareMap.get(DcMotor.class, "Front_left");
    Back_left = hardwareMap.get(DcMotor.class, "Back_left");
    extensionservo = hardwareMap.get(CRServo.class, "extension servo");
    topservo = hardwareMap.get(Servo.class, "top servo");
    rightservo = hardwareMap.get(Servo.class, "rightservo");
    leftservo = hardwareMap.get(Servo.class, "leftservo");

    waitForStart();
    
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_left.setDirection(DcMotor.Direction.REVERSE);
    Front_left.setDirection(DcMotor.Direction.REVERSE);
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        drone_launch();
        DriveTrain();
        Lift();
        Telemetry2();
        Arm();
        extension();
        clawservo();
        wrist();
        presets();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Lift() {
    if (gamepad2.x) {
      liftmotor.setPower(1);
    } else if (gamepad2.b) {
      liftmotor.setPower(-1);
    } else {
      liftmotor.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void extension() {
    extensionservo.setPower(-gamepad2.right_stick_y);
  }

  /**
   * Describe this function...
   */
  private void wrist() {
    if (gamepad2.y) {
      topservo.setPosition(1);
    } else if (gamepad2.a) {
      topservo.setPosition(0);
    }
  }

  /**
   * Describe this function...
   */
  private void clawservo() {
    //left servo up
                    if (gamepad2.left_bumper) {
                        leftservo.setPosition(1);
                        

                    }
                    //left servo down
                    if (gamepad2.left_trigger >0) {
                        leftservo.setPosition(0);

                    }


                   //right servo up
                   if(gamepad2.right_bumper) {
                       rightservo.setPosition(0);
                       
                   }
                   //right servo down
                   if (gamepad2.right_trigger > 0){
                        rightservo.setPosition(1);
                        
                    }
                    
  }

  /**
   * Describe this function...
   */
  private void DriveTrain() {
    float Strafe;
    float Turn;
    float forward;
    double denominator;

    Strafe = gamepad1.left_stick_x;
    Turn = -gamepad1.right_stick_x;
    forward = -gamepad1.left_stick_y;
    denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Turn + forward + Strafe));
    Back_left.setPower((forward + (Strafe - Turn)) / denominator);
    Front_left.setPower((forward - (Strafe + Turn)) / denominator);
    Back_right.setPower((forward - (Strafe - Turn)) / denominator);
    Front_right.setPower((forward + Strafe + Turn) / denominator);
  }
    
    private void presets() {
       //preset - zero
                    if (gamepad2.dpad_down) {
                  ArmTarget = 0;
                  
                }
                
                //preset - high
                    if (gamepad2.dpad_up) {
                  ArmTarget = -1200;
                 
                }
                
                //preset - pickup
                    if (gamepad2.dpad_left) {
                  ArmTarget = 0;
                 
                }
              
              //preset - trnsit
                    if (gamepad2.dpad_right) {
                  ArmTarget = -150;
                  
                }
      
         if (gamepad2.dpad_left) {
                  ArmTarget = -850;
                  
                }
      
    }
    
    
  /**
   * Describe this function...
   */
  private void Telemetry2() {
    telemetry.addData("BACK_LEFTPOWER", Back_left.getPower());
    telemetry.addData("BACK_RIGHTPOWER", Back_right.getPower());
    telemetry.addData("FRONT_RIGHTPOWER", Front_right.getPower());
    telemetry.addData("FRONT_LEFTPOWER", Front_left.getPower());
    telemetry.addData("ARM_POWER", arm.getPower());
    telemetry.addData("ARM_POS", arm.getCurrentPosition());
    telemetry.addData("Wrist POS", topservo.getPosition());
    telemetry.addData("ex", extensionservo.getPower());
    telemetry.addData("lservos", leftservo.getPosition());
    telemetry.addData("rservos", rightservo.getPosition());
    telemetry.addData("wrist", topservo.getPosition());
    telemetry.addData("drone_servo", droneservo.getPosition());
    telemetry.update();
  }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeleopProgram2 (Blocks to Java)")
public class TeleopProgram2 extends LinearOpMode {

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
  
  
  
  

  /**
   * Describe this function...
   */
  private void Arm() {
    arm.setPower(gamepad2.left_stick_y);
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
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Lift() {
    if (gamepad2.right_bumper) {
      liftmotor.setPower(1);
    } else if (gamepad2.left_bumper) {
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
    extensionservo.setPower(gamepad2.right_stick_y);
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
    /* boolean RightServo_setto0;
    boolean LeftServo_setto0;

    if (gamepad2.b) {
      if (RightServo_setto0) {
        rightservo.setPosition(0);
        RightServo_setto0 = false;
      } else {
        rightservo.setPosition(1);
        RightServo_setto0 = true;
      }
    }
    if (gamepad2.x) {
      if (LeftServo_setto0) {
        leftservo.setPosition(0);
        LeftServo_setto0 = false;
      } else {
        leftservo.setPosition(1);
        LeftServo_setto0 = true;
      }
    } */
  }

  /**
   * Describe this function...
   */
  private void DriveTrain() {
    float Strafe;
    float Turn;
    float forward;
    double denominator;

    Strafe = -gamepad1.left_stick_x;
    Turn = -gamepad1.right_stick_x;
    forward = -gamepad1.left_stick_y;
    denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Turn + forward + Strafe));
    Back_left.setPower((forward + (Strafe - Turn)) / denominator);
    Front_left.setPower((forward - (Strafe + Turn)) / denominator);
    Back_right.setPower((forward - (Strafe - Turn)) / denominator);
    Front_right.setPower((forward + Strafe + Turn) / denominator);
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

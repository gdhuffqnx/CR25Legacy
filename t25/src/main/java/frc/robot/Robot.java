// Copyright (2025)
// Control Freaks
// Vicksburg High School
// Michigan
// this is a time based program
//

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
//import e//du.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import com.studica.frc.AHRS;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

   private final XboxController m_driverController = new XboxController(0);
   private final XboxController m_bodyController   = new XboxController(1);

   private final DigitalInput handoffLimitSwitch = new DigitalInput(0);

   CANCoder frontRightEncoder = new CANCoder(1);
   CANCoder frontLeftEncoder = new CANCoder(0);
   CANCoder backRightEncoder = new CANCoder(3);
   CANCoder backLeftEncoder = new CANCoder(2);

   private boolean timeInit;
   private int counter;
   private int turnCompleteTimer;
   private double TIME_STEP;
   private double distance;
   private double distanceInit;
   private double yawInit;
   private double yawDesired;
   private double flcmd = 0;
   private double frcmd = 0;
   private double blcmd = 0;
   private double brcmd = 0;
   private double prevDesiredAngle;
   private double prevGain;
   private double prevDrCmd;
   private double winchPwr;
   private double prevWinchPwr;

   private double backLeftSpinAngle;
   private double backRightSpinAngle;
   private double frontLeftSpinAngle;
   private double frontRightSpinAngle;

   private double frontRightSpinErrorSum;
   private double backRightSpinErrorSum;
   private double frontLeftSpinErrorSum;
   private double backLeftSpinErrorSum;

   private double frontRightSpinError; 
   private double backRightSpinError;
   private double frontLeftSpinError;
   private double backLeftSpinError;

   private double backLeftDriveGain;
   private double backRightDriveGain;
   private double frontLeftDriveGain;
   private double frontRightDriveGain;

   public double yawFlipError;
   public double yawFlipErrorSum;
   public double prevYawFlipError;

   private double kp_yaw;
   private double ki_yaw;
   private double kd_yaw;
   
   private int state; 

   float yaw;
   int debug_timer;
   public int stateCounter;
   public int errorCounter;
   public int waitCounter;

   private boolean COLLECTOR_RAISED;
   private boolean COLLECTOR_LOWERED;
   private boolean ARM_RAISED;
   private boolean ARM_LOWERED;
   private boolean b_armPivot;
   private boolean b_collector;

   BooleanLogEntry myBooleanLog;
   DoubleLogEntry myDoubleLog;
   StringLogEntry myStringLog;

   //AHRS gyro = new AHRS(SPI.Port.kMXP);
   AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

   private final SparkMax m_frontLeftDrive  = new SparkMax(1, MotorType.kBrushless);
   private final SparkMax m_frontRightDrive = new SparkMax(3, MotorType.kBrushless);
   private final SparkMax m_backLeftDrive   = new SparkMax(5, MotorType.kBrushless);
   private final SparkMax m_backRightDrive  = new SparkMax(7, MotorType.kBrushless);

   private final SparkMax m_frontLeftSteer  = new SparkMax(2,MotorType.kBrushless);
   private final SparkMax m_frontRightSteer = new SparkMax(4, MotorType.kBrushless);
   private final SparkMax m_backLeftSteer   = new SparkMax(6,MotorType.kBrushless);
   private final SparkMax m_backRightSteer  = new SparkMax(8, MotorType.kBrushless);

   private final SparkMax m_handoff = new SparkMax(9,MotorType.kBrushless); 
   private final SparkMax m_collector = new SparkMax(10,MotorType.kBrushless);
   private final SparkMax m_shooterIndexer = new SparkMax(11,MotorType.kBrushless); 
   // CAN ID 12 is the pneumatics hub
   private final SparkMax m_shooterFlywheel = new SparkMax(13,MotorType.kBrushless);
   private final SparkMax m_winchOne = new SparkMax(14,MotorType.kBrushless);
   private final SparkMax m_winchTwo = new SparkMax(15,MotorType.kBrushless);

   private RelativeEncoder e_frontLeftDrive;
   private RelativeEncoder e_frontRightDrive;
   private RelativeEncoder e_backLeftDrive;
   private RelativeEncoder e_backRightDrive;

   AddressableLED m_led = new AddressableLED(9);
   AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

   UsbCamera camera1;
   UsbCamera camera2;

   //  private final String autoStraight= "straight"; 
   //  private final String autoDiagonal="diagonal";
   //  private final String autoSleep="sleep";
   //  private String autoSelected;
   //  private  Sendable Chooser<String> autoChoose;
   //  autoChoose = new SendableChooser<>;

   @Override
   public void robotInit() {
      

      m_led.setLength(m_ledBuffer.getLength());
      m_led.setData(m_ledBuffer);
      m_led.start();

      // yaw PID values used in autonomous and teleop
      kp_yaw = 0.009;
      ki_yaw = 0.0001;
      kd_yaw = 0.0001;
      
      // important constant
      TIME_STEP = 0.02;  // 50 hertz

      // Start camera
      camera1 = CameraServer.startAutomaticCapture(0);
      camera2 = CameraServer.startAutomaticCapture(1);

      m_frontLeftDrive.setInverted(true);
      m_frontRightDrive.setInverted(true);
      m_backLeftDrive.setInverted(true);  
      m_backRightDrive.setInverted(true);  

      m_collector.setInverted(false); 
      m_handoff.setInverted(true); 
      m_shooterFlywheel.setInverted(true); 
      m_shooterIndexer.setInverted(true);

      frontLeftEncoder.setPosition(0);
      frontRightEncoder.setPosition(0);
      backLeftEncoder.setPosition(0);
      backRightEncoder.setPosition(0);  
      
      // initializing global variables to zero
      frontRightSpinErrorSum = 0;
      backRightSpinErrorSum = 0;
      frontLeftSpinErrorSum = 0;
      backLeftSpinErrorSum = 0;

      frontRightSpinError = 0;
      backRightSpinError = 0;
      frontLeftSpinError = 0;
      backLeftSpinError = 0;

      yawInit = 0.0;
      yawDesired = 0.0;
      yawFlipError = 0.0;
      yawFlipErrorSum = 0.0;
      prevYawFlipError = 0.0;
      turnCompleteTimer = 0;
      timeInit = false;
      counter = 0;
      distance = 0;
      yaw = 0;
      prevDesiredAngle = 0;
      prevGain = 0;
      stateCounter = 0;
      errorCounter = 0;
      prevDrCmd = 0;
      debug_timer = 0;
	   winchPwr = 0;
	   prevWinchPwr = 0;
      waitCounter = 0;

//test variables for autonoumous initilizer 
      //   autoChoose.setDefaultOption("straight", autoStraight);
    //autoChoose.addOption("diagonal", autoDiagonal);
    // autoChoose.addOption("sleep", autoSleep);
    //SmartDashboard.putData("Auto choices", autoChoose);
      // Initialize the gyro in robot init
      gyro.reset();

      // assign motor encoders
	   e_frontLeftDrive  = m_frontLeftDrive.getEncoder();
      e_frontRightDrive = m_frontRightDrive.getEncoder();
      e_backLeftDrive   = m_backLeftDrive.getEncoder();
      e_backRightDrive  = m_backRightDrive.getEncoder();

      // Starts recording to data log
      DataLogManager.start();
      DataLog log = DataLogManager.getLog();
      myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
      myDoubleLog = new DoubleLogEntry(log, "/my/double");
      myStringLog = new StringLogEntry(log, "/my/string");
   }

   @Override
   public void autonomousInit() {
      state = 0;
      timeInit = false;
   //      autoSelected = autoChoose.getSelected();
   //  System.out.println("Auto selected: " + autoSelected);
   //        boolean sleep=false;
   }

   @Override
   public void autonomousPeriodic() {
      yaw =gyro.getYaw();

      //double backwardsDistance;
      //double forwardMini;
      double forwardDistance;
      //boolean sleep;
      /*    switch (m_autoSelected) {
      case autoDiagonal :
      double backwardsDistance = 48 ;
      double forwardMini =4 ;
      double forwardDistance = 44;
       break;
        case autoSleep:
      double backwardsDistance = -76;
      double forwardMini = 4 ;
      double forwardDistance =72;
      boolean sleep=true;
        break;
      case autoStraight:
      default:*/
      //backwardsDistance = -36;
      //forwardMini = 4 ;
      forwardDistance =32;
      //sleep=false;
      //break;
          

      switch(state) {
         case 0: 
            if (shootRing()){
               m_shooterIndexer.set(0);
               m_shooterFlywheel.set(0);
               state++;
            }
         break;
         case 1: 
            if (turnDegrees(20,0.25)) {
               state++;
            }
         break;
         case 2: 
            if (driveForwardInches(forwardDistance,0.25)) {
               state++;
            }
         break;
         case 3:

         break;
         
      }
   } // end of autonomousPeriodic

   @Override
   public void teleopPeriodic() {
      double orientation = 1;
      double gain;
      double drCmd;
      
      double desiredAngle;
      double desiredAngleFlipped;
      double filteredAngle;
      double joyXPos;
      double joyYPos;
      boolean leftBumper;
      boolean rightBumper;
      double d_yaw;
      double other_yaw;
      double gainTgt;
      double togglePad;
      boolean yawCompensationEnable = false;
      boolean buttonA;
      boolean buttonB;
      boolean buttonX;
      boolean buttonY; 
      double desiredYaw;
      double currentAngle;

      boolean button2A; 
      boolean button2B;
      boolean button2Y;
      boolean button2X; 
      boolean left2Bumper;
      boolean right2Bumper;
      double leftTrigger;
      double rightTrigger;

      //Game Pad 1 - Driver
      joyXPos = m_driverController.getRightX();
      joyYPos = m_driverController.getRightY();
      leftBumper = m_driverController.getLeftBumper();
      rightBumper = m_driverController.getRightBumper();
      togglePad = m_driverController.getLeftX();
      buttonA = m_driverController.getAButton();
      buttonB = m_driverController.getBButton();
      buttonX = m_driverController.getXButton();
      buttonY = m_driverController.getYButton();
      leftTrigger = m_driverController.getLeftTriggerAxis();
      rightTrigger = m_driverController.getRightTriggerAxis();

	  //Game Pad 2 - Arm control
      double right2Trigger= m_bodyController.getRightTriggerAxis();
      button2A = m_bodyController.getAButtonPressed();
      button2B = m_bodyController.getBButton();
      button2Y = m_bodyController.getYButton();
      button2X = m_bodyController.getXButtonPressed();
      left2Bumper = m_bodyController.getLeftBumper();
      right2Bumper = m_bodyController.getRightBumper();
      desiredYaw = 0;
      blcmd = 0.0;
      flcmd = 0.0;
      frcmd = 0.0;
      brcmd = 0.0;
      drCmd = 0.0;
      yaw = gyro.getYaw();
      desiredAngle = prevDesiredAngle;
	  
	   // Ring Collection and handoff code
      if (button2B) {
         pickupRing();
      }

      //Turn off collection with the handoff limit switch
      if (!handoffLimitSwitch.get()) {
         m_collector.set(0);
         m_handoff.set(0);

      }

      if (left2Bumper ) {
         m_shooterFlywheel.set(1.0);
      } else {
         m_shooterFlywheel.set(0);
      }

      if (right2Trigger>.6) {
         m_shooterIndexer.set(1.0);
      } else {
         m_shooterIndexer.set(0);
      }

      //  m_ledBuffer.setRGB(100, 255, 0, 0);
      // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
         // Sets the specified LED to the RGB values for red
      //   m_ledBuffer.setRGB(i, 255, 0, 0);
      //}
      //m_led.setData(m_ledBuffer);

      //this gain is the drive wheel speed gain
      gainTgt = 0.25; 
      if (leftBumper) {
         gainTgt = 0.8;
      } else if (rightBumper) {
         gainTgt = 0.1;
      }
      gain = prevGain + (0.025*(gainTgt - prevGain));
      prevGain = gain;
      gain = gain * orientation;

      // if the driver joystick is not near zero, drive
      if ((Math.abs(joyXPos) > 0.09)||(Math.abs(joyYPos) > 0.09)){
         drCmd = (joyXPos*joyXPos)+(joyYPos*joyYPos);
         drCmd = gain * Math.sqrt(drCmd);
         // this is a large function that determines the desired
         //angle of the wheel
         desiredAngle = determineDesiredAngle(joyXPos,joyYPos);    
      }

      // these are offsets
      backLeftSpinAngle = 0;
      backRightSpinAngle =0;
      frontLeftSpinAngle =0;
      frontRightSpinAngle =0;

      // this robot tends to favor the right side motors
      //slow them down a little
      backLeftDriveGain =1;
      backRightDriveGain = 0.8;
      frontLeftDriveGain= 1;
      frontRightDriveGain = 0.8;
      
      // rotate the robot
      if ((togglePad) > 0.75){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = 0.5 * orientation * Math.abs(gain);
      }
      if ((togglePad) < -0.75){
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = -0.5 * orientation * Math.abs(gain);
      }
      
      // if using the buttons to drive straight, use the following logic
      double MAX_BUTTON_SPEED = 0.80;
      double MIN_BUTTON_SPEED = 0.15;
      if (buttonA ) {
         desiredAngle = 0;
         backLeftSpinAngle = 1;
         backRightSpinAngle = 1;
         frontLeftSpinAngle =1;
         frontRightSpinAngle =1;
         backLeftDriveGain =-1;
         backRightDriveGain =-1;
         frontLeftDriveGain= -1;
         frontRightDriveGain =-1;
         if (leftBumper){
            if (prevDrCmd < MAX_BUTTON_SPEED){
               drCmd = prevDrCmd +.01;
            } else {
               drCmd = MAX_BUTTON_SPEED;
            }
         } else {
            if (prevDrCmd > MIN_BUTTON_SPEED){
               drCmd = prevDrCmd -.01;
            } else {
               drCmd = MIN_BUTTON_SPEED;
            }
         }
      }
      if (buttonB) {
         desiredAngle = 0;
         backLeftSpinAngle = 90;
         backRightSpinAngle = 90;
         frontLeftSpinAngle =90;
         frontRightSpinAngle =90;
         backLeftDriveGain =-1;
         backRightDriveGain =-1;
         frontLeftDriveGain= -1;
         frontRightDriveGain = -1;

         if (leftBumper){
            if (prevDrCmd < MAX_BUTTON_SPEED){
               drCmd = prevDrCmd +.01;
            } else {
               drCmd = MAX_BUTTON_SPEED;
            }
         } else {
            if (prevDrCmd>.1){
               drCmd = prevDrCmd -.01;
            } else {
               drCmd = MIN_BUTTON_SPEED;
            }
         }
      }
      if (buttonX) {
         desiredAngle = 0;
         backLeftSpinAngle = 90;
         backRightSpinAngle = 90;
         frontLeftSpinAngle =90;
         frontRightSpinAngle =90;
         backLeftDriveGain =1;
         backRightDriveGain =1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = 1;
         if (leftBumper) {
            if (prevDrCmd < MAX_BUTTON_SPEED){
               drCmd = prevDrCmd +.01;
            } else {
               drCmd = MAX_BUTTON_SPEED;
            }
         } else {
            if (prevDrCmd > MIN_BUTTON_SPEED) {
               drCmd = prevDrCmd -.01;
            } else {
              drCmd = MIN_BUTTON_SPEED;
            }
         }
      }
      if (buttonY) {
         desiredAngle = 0;
         backLeftSpinAngle = 1;
         backRightSpinAngle = 1;
         frontLeftSpinAngle =1;
         frontRightSpinAngle =1;
         backLeftDriveGain =1;
         backRightDriveGain =1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = 1;
         if (leftBumper){
            if (prevDrCmd < MAX_BUTTON_SPEED){
               drCmd = prevDrCmd +.01;
            } else {
               drCmd = MAX_BUTTON_SPEED;
            } 
         } else {
            if (prevDrCmd > MIN_BUTTON_SPEED){
               drCmd = prevDrCmd -.01;
            } else {
               drCmd = MIN_BUTTON_SPEED;
            }
         }
      }
      // defensive position
      if (m_driverController.getPOV()>45&&m_driverController.getPOV()<135){
         desiredAngle = 0;
         backLeftSpinAngle = -45;
         backRightSpinAngle = 45;
         frontLeftSpinAngle =45;
         frontRightSpinAngle =-45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;

         drCmd = 0.0;
      }

      double bodyControllerPOV;
      bodyControllerPOV =  m_bodyController.getPOV();
	   if ((bodyControllerPOV > 260) && (bodyControllerPOV < 280)){

         SmartDashboard.putNumber("POV", bodyControllerPOV);
     }
     if ((bodyControllerPOV > 80) && (bodyControllerPOV < 100)){

        SmartDashboard.putNumber("POV", bodyControllerPOV);
     }

      double MAX_NEGATIVE_PWR = -0.8;
      double MAX_POSITIVE_PWR =  0.8;
      
      if ((170 < bodyControllerPOV)&&(bodyControllerPOV<190 )){
         if (prevWinchPwr >  MAX_NEGATIVE_PWR){
            winchPwr = prevWinchPwr -0.01;
         } else {
            winchPwr = MAX_NEGATIVE_PWR;
         }  
      } else if (( bodyControllerPOV<10)||( bodyControllerPOV>350)){
         if (prevWinchPwr < MAX_POSITIVE_PWR){
            winchPwr = prevWinchPwr +0.01;
         } else {
            winchPwr = MAX_POSITIVE_PWR;
         }
      } else {
         winchPwr = 0.0;
      }
  	   m_winchOne.set(winchPwr);
      m_winchTwo.set(winchPwr);
      prevWinchPwr = winchPwr; 
	  
      // next two sections deal with rotating the robot 180 degrees
      // turning it around - yaw
      other_yaw = Double.valueOf(yaw);
      
      if (rightTrigger > 0.5) {
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;
         desiredYaw = 180.0;
         if (other_yaw < 0) {
            other_yaw = other_yaw + 360;
         }
         yawFlipError = desiredYaw-other_yaw;
         yawFlipErrorSum = yawFlipErrorSum + (yawFlipError * TIME_STEP);
         drCmd = (kp_yaw*yawFlipError)+(ki_yaw*yawFlipErrorSum)+(kd_yaw*(yawFlipError-prevYawFlipError));
         if (drCmd > 0.3) {drCmd = 0.3;}
         if (drCmd < -0.3) {drCmd = -0.3;}
         prevYawFlipError = yawFlipError;
      }
      else if (leftTrigger > 0.5) {
         desiredAngle = 0;
         backLeftSpinAngle = 45;
         backRightSpinAngle = -45;
         frontLeftSpinAngle =-45;
         frontRightSpinAngle =45;
         backLeftDriveGain =1;
         backRightDriveGain =-1;
         frontLeftDriveGain= 1;
         frontRightDriveGain = -1;
         desiredYaw = 0.0;
         yawFlipError = desiredYaw-other_yaw;
         yawFlipErrorSum = yawFlipErrorSum + (yawFlipError * TIME_STEP);

         drCmd = (kp_yaw*yawFlipError)+(ki_yaw*yawFlipErrorSum)+(kd_yaw*(yawFlipError-prevYawFlipError));
         if (drCmd > 0.3) {drCmd = 0.3;}
         if (drCmd < -0.3) {drCmd = -0.3;}
         prevYawFlipError = yawFlipError;
      } else {
         yawFlipErrorSum = 0.0;
         prevYawFlipError = 0.0;
      }

      d_yaw = Double.valueOf(yaw);
      //ZERO YAW
      /*if (buttonB && (drCmd < 0.03)) {
         desiredAngle = 0;
         backLeftSpinAngle   =  45;
         backRightSpinAngle  = -45;
         frontLeftSpinAngle  = -45;
         frontRightSpinAngle =  45;
         backLeftDriveGain   =  1;
         backRightDriveGain  = -1;
         frontLeftDriveGain  =  1;
         frontRightDriveGain = -1;

         drCmd = (desiredYaw-d_yaw)*kp_yaw * orientation;
         if (drCmd > 0.4) {drCmd = 0.4;}
         if (drCmd < -0.4) {drCmd = -0.4;}
      }*/

      // yaw compensation logic, if driving straight, try to zero yaw
      // can be disabled if yawCompensationEnable is set to false
      //
      if ((rightTrigger < 0.4) && (leftTrigger < 0.4)) {
         if (d_yaw > 20) {d_yaw = 20;}
         if (d_yaw < -20) {d_yaw = -20;}
	      if ((-10 < desiredAngle) && (desiredAngle < 10) && (drCmd > 0.1)&&yawCompensationEnable) {
            if (yaw > 1) {
		         backRightDriveGain  = backRightDriveGain * (1 + (d_yaw*0.1));
		         frontRightDriveGain = frontRightDriveGain * (1 + (d_yaw*0.1));
		      }
         }
	      if (((-170 > desiredAngle) || (desiredAngle > 170)) && (drCmd > 0.1)&&yawCompensationEnable) {
	         if (yaw < -1) {
	            backRightDriveGain  = backRightDriveGain * (1 - (d_yaw*0.1));
   	         frontRightDriveGain = frontRightDriveGain * (1 - (d_yaw*0.1));
		      }
         }
      }
	  
      currentAngle = frontLeftEncoder.getPosition();
      desiredAngle = angleWrap(currentAngle, desiredAngle);

      // Here is the logic that allows the wheels to decide whether 
      // to just spin backward or completely flip around
       if (desiredAngle >= -0.01) {
         desiredAngleFlipped = desiredAngle - 180;
         if (desiredAngleFlipped < -180) {
            desiredAngleFlipped = -180;
         } 
      } else {
         desiredAngleFlipped = desiredAngle + 180; 
      }

      if (Math.abs(frontLeftEncoder.getPosition() - desiredAngle)
        > Math.abs(frontLeftEncoder.getPosition() - desiredAngleFlipped)) 
      {
         desiredAngle = desiredAngleFlipped;
         backLeftDriveGain   = -backLeftDriveGain;
         backRightDriveGain  = -backRightDriveGain;
         frontLeftDriveGain  = -frontLeftDriveGain;
         frontRightDriveGain = -frontRightDriveGain;
      }  

      // simple filter to smooth out angle change, this may have a minor bug
      filteredAngle = desiredAngle;//prevDesiredAngle + (0.01*(desiredAngle - prevDesiredAngle));

      //
      // wheel spin
      // error = desired - current
      //
      frontRightSpinError = filteredAngle+frontRightSpinAngle-frontRightEncoder.getPosition();
      frontLeftSpinError = filteredAngle+frontLeftSpinAngle-frontLeftEncoder.getPosition();
      backRightSpinError = filteredAngle+backRightSpinAngle-backRightEncoder.getPosition();
      backLeftSpinError = filteredAngle+backLeftSpinAngle-backLeftEncoder.getPosition();
     
      //
      // needed for I term which isn't essential for wheel spin control
      //
      frontRightSpinErrorSum = 0.0;//frontRightSpinErrorSum+frontRightSpinError*TIME_STEP;
      backRightSpinErrorSum = 0.0;//backRightSpinErrorSum+backRightSpinError*TIME_STEP;
      frontLeftSpinErrorSum = 0.0;//frontLeftSpinErrorSum+frontLeftSpinError*TIME_STEP;
      backLeftSpinErrorSum = 0.0;//backLeftSpinErrorSum+backLeftSpinError*TIME_STEP;

      // calculate the required motor power command to get to desired wheel angle
      flcmd = wheelAngle(frontLeftSpinError,  frontLeftSpinErrorSum);
      frcmd =  wheelAngle(frontRightSpinError, frontRightSpinErrorSum);       
      blcmd = wheelAngle(backLeftSpinError,   backLeftSpinErrorSum);
      brcmd = wheelAngle(backRightSpinError,  backRightSpinErrorSum);
      
      prevDesiredAngle = filteredAngle;

      // sending the power commands to the driving and steering motors
      prevDrCmd = drCmd;

      m_frontLeftDrive.set(drCmd*frontLeftDriveGain);
      m_frontRightDrive.set(drCmd*frontRightDriveGain);
      m_backLeftDrive.set(drCmd*backLeftDriveGain);
      m_backRightDrive.set(drCmd*backRightDriveGain);
	  
      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);
	  
      // telemtry information, feel free to change as needed
      //SmartDashboard.putNumber("Back Left Encoder", backLeftEncoder.getPosition());
      //SmartDashboard.putNumber("Back Right Encoder", backRightEncoder.getPosition());  

      if (debug_timer >= 9)
      { 
        
         //SmartDashboard.putNumber("Back left spin angle want zero", backLeftSpinAngle);
         //SmartDashboard.putNumber("Filtered Angle", filteredAngle);
         //myBooleanLog.append(armPivotLimitSwitch);
         //myDoubleLog.append(desiredWinchTarget);
         debug_timer = 0;

      } else {
         debug_timer = debug_timer +1;
      }   
   }
//
// Drive Forward Inches
// used in autonomous mode
//
   public boolean driveForwardInches(double inches, double powerInput) {
      boolean complete = false; 
      double driveYaw;
      double leftPwr;
      double rightPwr;
      double power = 0;
      
      rightPwr = 0.0;
      leftPwr = 0.0;
      driveYaw = Double.valueOf(yaw);

      distance = e_frontLeftDrive.getPosition();
      if (timeInit == false)  {
         distanceInit = distance;
         timeInit = true;
      }
      //
      // wheel spin
      // error = desired - current
      //
      // target is 0
      frontRightSpinError = 0 - frontRightEncoder.getPosition();
      backRightSpinError  = 0 - backRightEncoder.getPosition();
      frontLeftSpinError  = 0 - frontLeftEncoder.getPosition();
      backLeftSpinError   = 0 - backLeftEncoder.getPosition();
     
      //
      // needed for I term which isn't essential for wheel spin control
      //
      frontRightSpinErrorSum = 0.0; //frontRightSpinErrorSum+frontRightSpinError*TIME_STEP;
      backRightSpinErrorSum = 0.0;  //backRightSpinErrorSum+backRightSpinError*TIME_STEP;
      frontLeftSpinErrorSum = 0.0;  //frontLeftSpinErrorSum+frontLeftSpinError*TIME_STEP;
      backLeftSpinErrorSum = 0.0;   //backLeftSpinErrorSum+backLeftSpinError*TIME_STEP;

      // calculate the required motor power command to get to desired wheel angle
      flcmd = wheelAngle(frontLeftSpinError,  frontLeftSpinErrorSum);
      frcmd = wheelAngle(frontRightSpinError, frontRightSpinErrorSum);       
      blcmd = wheelAngle(backLeftSpinError,   backLeftSpinErrorSum);
      brcmd = wheelAngle(backRightSpinError,  backRightSpinErrorSum);

      m_frontLeftSteer.set(flcmd);
      m_frontRightSteer.set(frcmd);
      m_backLeftSteer.set(blcmd);
      m_backRightSteer.set(brcmd);

      if ((distance - distanceInit) < (inches*0.026)) {   //was 0.56
         power = prevDrCmd + (0.1*(powerInput-prevDrCmd));
         //if (driveYaw > 2) {
         //   rightPwr = 0.025*(driveYaw);
         //} else if (driveYaw<-2){
         //   leftPwr = -0.025*(driveYaw);
        // }
         // this robot tends to favor the right side motors
         //slow them down a little
         backLeftDriveGain   = 1;
         backRightDriveGain  = 1;
         frontLeftDriveGain  = 1;
         frontRightDriveGain = 1;

         flcmd = frontLeftDriveGain*power+leftPwr;
         frcmd = frontRightDriveGain*power+rightPwr;
         blcmd = backLeftDriveGain*power+leftPwr;
         brcmd = backRightDriveGain*power+rightPwr;
         prevDrCmd = power;

         m_frontLeftDrive.set(flcmd);
         m_frontRightDrive.set(frcmd);
         m_backLeftDrive.set(blcmd);
         m_backRightDrive.set(brcmd);
      } else {
         complete = true; 
         timeInit = false; 
         m_frontLeftDrive.set(0.0);
         m_frontRightDrive.set(0.0);
         m_backLeftDrive.set(0.0);
         m_backRightDrive.set(0.0);
      }
     // telemtry
     if (counter > 9)
     {
      //distance
      SmartDashboard.putNumber("Power", power);
      SmartDashboard.putNumber("Distance", distance);
      SmartDashboard.putNumber("D Init", distanceInit);
        //myDoubleLog.append(rightPwr);
        //myDoubleLog.append(leftPwr);
        //myDoubleLog.append(driveYaw);
        counter = 0;
     } else {
        counter = counter +1;
     }
     return(complete);
   }

// Drive Forward Inches
// used in autonomous mode
//
public boolean turnDegrees(double desiredDegrees, double powerInput) {
   //clockwise motion for positive desiredDegrees
   //counter clockwise motion for negative desiredDegrees
   //powerInput should be positive
   boolean complete = false; 
   double driveYaw;
   double power = 0;

   double ERROR_TOLERANCE = 5.0;
   
   driveYaw = Double.valueOf(yaw) + 720;
   
   //if (driveYaw < 0) {
   //   driveYaw = driveYaw + 360;
  // }
   if (powerInput < 0) {
      powerInput = 0.0; //only use if positive
   }

   if (timeInit == false)  {
      yawInit = driveYaw;
      yawDesired = driveYaw + desiredDegrees;
      timeInit = true;
      turnCompleteTimer = 0;
   }

   backLeftDriveGain =1;
   backRightDriveGain =-1;
   frontLeftDriveGain= 1;
   frontRightDriveGain = -1;

   yawFlipError = yawDesired-driveYaw;
   yawFlipErrorSum = yawFlipErrorSum + (yawFlipError * TIME_STEP);
   SmartDashboard.putNumber("state", state);
   SmartDashboard.putNumber("yawFlipError", yawFlipError);
   SmartDashboard.putNumber("driveYaw", driveYaw);
   if (Math.abs(yawFlipError) > ERROR_TOLERANCE) { 
      backLeftSpinAngle   =  45;
      backRightSpinAngle  = -45;
      frontLeftSpinAngle  = -45;
      frontRightSpinAngle =  45;
      

      power = (kp_yaw*yawFlipError)+(ki_yaw*yawFlipErrorSum)+(kd_yaw*(yawFlipError-prevYawFlipError));
      if (power > powerInput) {power = powerInput;}
      if (power < -(powerInput)) {power = -(powerInput);}

      prevYawFlipError = yawFlipError;

      m_frontLeftDrive.set(power*frontLeftDriveGain);
      m_frontRightDrive.set(power*frontRightDriveGain);
      m_backLeftDrive.set(power*backLeftDriveGain);
      m_backRightDrive.set(power*backRightDriveGain);
   } else {
      backLeftSpinAngle   = 0;
      backRightSpinAngle  = 0;
      frontLeftSpinAngle  = 0;
      frontRightSpinAngle = 0;

      m_frontLeftSteer.set(0);
      m_frontRightSteer.set(0);
      m_backLeftSteer.set(0);
      m_backRightSteer.set(0);
      m_frontLeftDrive.set(0);
      m_frontRightDrive.set(0);
      m_backLeftDrive.set(0);
      m_backRightDrive.set(0);

      turnCompleteTimer = turnCompleteTimer + 1;

      if (turnCompleteTimer > 20){
         complete = true;
         timeInit = false;
         turnCompleteTimer = 0;
      }
   }

   //
   // still in turn degrees, control turning motors

   frontRightSpinError = frontRightSpinAngle - frontRightEncoder.getPosition();
   backRightSpinError  = backRightSpinAngle - backRightEncoder.getPosition();
   frontLeftSpinError  = frontLeftSpinAngle - frontLeftEncoder.getPosition();
   backLeftSpinError   = backLeftSpinAngle - backLeftEncoder.getPosition();
  
   //
   // needed for I term which isn't essential for wheel spin control
   //
   frontRightSpinErrorSum = 0.0; //frontRightSpinErrorSum+frontRightSpinError*TIME_STEP;
   backRightSpinErrorSum = 0.0;  //backRightSpinErrorSum+backRightSpinError*TIME_STEP;
   frontLeftSpinErrorSum = 0.0;  //frontLeftSpinErrorSum+frontLeftSpinError*TIME_STEP;
   backLeftSpinErrorSum = 0.0;   //backLeftSpinErrorSum+backLeftSpinError*TIME_STEP;

   // calculate the required motor power command to get to desired wheel angle
   flcmd = wheelAngle(frontLeftSpinError,  frontLeftSpinErrorSum);
   frcmd =  wheelAngle(frontRightSpinError, frontRightSpinErrorSum);       
   blcmd = wheelAngle(backLeftSpinError,   backLeftSpinErrorSum);
   brcmd = wheelAngle(backRightSpinError,  backRightSpinErrorSum);

   m_frontLeftSteer.set(flcmd);
   m_frontRightSteer.set(frcmd);
   m_backLeftSteer.set(blcmd);
   m_backRightSteer.set(brcmd);

   return(complete);
}

   //
   // Wheel Angle - PI control of wheel angle
   // used in teleop mode
   // outputs the power command for the steer wheel
   //
   public double wheelAngle(double error, double errorSum)
   {
      double powerCmd;
      double kp = 0.008;//0.008
      double ki = 0.00;
      double maxPwr = 0.5;

      powerCmd = (kp * error) + (ki * errorSum);

      if (powerCmd > maxPwr){powerCmd = maxPwr;}
      if (powerCmd < -maxPwr){powerCmd = -maxPwr;}

      return(powerCmd);
   }
   //
   // Angle Angle - attempts to set the desired gear in the same
   // 0-360 or 360-720 etc range
   // used in teleop mode
   // NOTE:  THIS COULD BE IMPROVED
   //
   public static double angleWrap(double current, double desired)
   {
      if ((current < -450)) {
         if (desired > 90) {
            return(desired - 720);
         }
      }
      if ((current > 450)) {
         if (desired < -90) {
            return(desired + 720);
         }
      }
      if ((current > 90)) {
         if (desired < -90) {
            return(desired + 360);
         }
      }
      if ((current < -90)) {
         if (desired > 90) {
            return(desired - 360);
         }
      }
      return(desired);
   }

   // take the joystick angle and convert it to degrees 
   public static double determineDesiredAngle(double joyX, double joyY) {   
      if ((joyX > 0.1)&&(joyY < -0.1)) {
         return(-57.3*Math.atan(Math.abs(joyX/joyY)));
      }
      if ((joyX > 0.1)&&(Math.abs(joyY) < 0.1)) {
         return(-90);
      }
      if ((joyX < -0.1)&&(Math.abs(joyY) < 0.1)) {
         return(90);
      }
      if ((Math.abs(joyX) < 0.1)&&(joyY < -0.1)) {
         return(0.1);
      }
      if ((Math.abs(joyX) < 0.1)&&(joyY > 0.1)) {
         return(180);
      }
      if ((joyX > 0.1)&&(joyY > 0.1)) {
         return(-180+57.3*Math.atan(Math.abs(joyX/joyY)));
      }
      if ((joyX < -0.1)&&(joyY < -0.1)) {
         return(57.3*Math.atan(Math.abs(joyX/joyY)));
      }
      if ((joyX < -0.1)&&(joyY > 0.1)) {
         return(180-57.3*Math.atan(Math.abs(joyX/joyY)));
      }
      return(0.1);
   }
   
   // Raise or lower the main arm   
   private void armPivot() {

   }
   
   // Power on the collector and handoff device to grab a ring
   public void pickupRing() {
      m_collector.set(.2);
      m_handoff.set(0.25);
      

   }
   
   public boolean shootRing() {
      boolean complete=false;
      m_shooterFlywheel.set(1.0);
      waitCounter = waitCounter++;
      if(waitCounter > 50) {
         m_shooterIndexer.set(.7);
         if (waitCounter> 100){
            complete= true;
            waitCounter = 0;
            m_shooterIndexer.set(0);
            m_shooterFlywheel.set(0);
         }
      }
        return (complete);
   }   //END
}
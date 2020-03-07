/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.io.*;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

//ultrasound
import edu.wpi.first.wpilibj.Ultrasonic;
//import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
//Gyro
//import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.SPI.Port;

//import edu.wpi.first.wpilibj.templates;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {

  // All eight positions of the lower chassie //even on right odd on left
  private static final int kFrontLeftChannel1 = 16;
  private static final int kFrontLeftChannel2 = 15;

  private static final int kRearLeftChannel1 = 1;
  private static final int kRearLeftChannel2 = 2;

  private static final int kFrontRightChannel1 = 14;
  private static final int kFrontRightChannel2 = 13;

  private static final int kRearRightChannel1 = 3;
  private static final int kRearRightChannel2 = 4;

  private static final int mastExtensionChannel = 11;

  int i = 0;
  private static final int kJoystickChannelUpper = 0;
  private static final int kJoystickChannelLower = 1;

  // Climber variables

  private static final int climb1 = 5;
  private static final int climb2 = 6;

  // Shooter
  private static final int shooter1 = 8;
  private static final int shooter2 = 9;

  private static final int shooterFeeder = 7;
  // intake
  private static final int intake = 10;

  // mast
  private static final int mast = 11;
  // lower chassie drive
  private MecanumDrive m_robotDrive;
  
// AHRS ahrs = new AHRS(SerialPort.Port );

  // Gyroscope variables
  Gyro gyroSPI = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  double current_angle;
  double Kp = 0.03;

  Ultrasonic dist = new Ultrasonic(1, 2);
  // Servo Stuff
  Servo ballStop = new Servo(0);
  Servo cameraMount = new Servo(1);

  // Joystick defs
  private Joystick xbox_lower;
  private Joystick xbox_upper;

  // Xbox input variables
  // upper joystick
  double x_left_trigger_upper, x_right_trigger_upper; // triggers on xbox
  double x_leftY_upper;
  double x_leftX_upper;
  double x_rightY_upper;
  double x_rightX_upper;
  boolean x_right_bumper_upper, x_left_bumper_upper; // bumper on xbox
  boolean x_back_button_upper, x_start_button_upper;
  boolean x_Xbutton_upper, x_Ybutton_upper;
  boolean x_aButton_upper, x_bButton_upper;

  // lower joystick
  double x_left_trigger_lower, x_right_trigger_lower; // triggers on xbox
  double x_leftY_lower;
  double x_leftX_lower;
  double x_rightY_lower;
  double x_rightX_lower;
  boolean x_right_bumper_lower, x_left_bumper_lower; // bumper on xbox
  boolean x_back_button_lower, x_start_button_lower;
  boolean x_Xbutton_lower, x_Ybutton_lower;
  boolean x_aButton_lower, x_bButton_lower;
  // motors
  WPI_TalonSRX M_intake = new WPI_TalonSRX(intake);
  WPI_TalonSRX frontLeft1 = new WPI_TalonSRX(kFrontLeftChannel1);
  WPI_TalonSRX rearLeft1 = new WPI_TalonSRX(kRearLeftChannel1);
  WPI_TalonSRX frontRight1 = new WPI_TalonSRX(kFrontRightChannel1);
  WPI_TalonSRX rearRight1 = new WPI_TalonSRX(kRearRightChannel1);
  WPI_TalonSRX frontLeft2 = new WPI_TalonSRX(kFrontLeftChannel2);
  WPI_TalonSRX rearLeft2 = new WPI_TalonSRX(kRearLeftChannel2);
  WPI_TalonSRX frontRight2 = new WPI_TalonSRX(kFrontRightChannel2);
  WPI_TalonSRX rearRight2 = new WPI_TalonSRX(kRearRightChannel2);
  WPI_TalonSRX mastExtension1 = new WPI_TalonSRX(mastExtensionChannel);
  WPI_TalonSRX M_climb1 = new WPI_TalonSRX(climb1);
  WPI_TalonSRX M_climb2 = new WPI_TalonSRX(climb2);
  WPI_TalonSRX M_shooter1 = new WPI_TalonSRX(shooter1);
  WPI_TalonSRX M_shooter2 = new WPI_TalonSRX(shooter2);
  WPI_TalonSRX M_shooterFeeder = new WPI_TalonSRX(shooterFeeder);
  WPI_TalonSRX M_mast = new WPI_TalonSRX(mast);

  // All Speed Control groups
  // climber is a group.
  SpeedControllerGroup S_climb = new SpeedControllerGroup(M_climb1, M_climb2);
  // lower chassie groups
  SpeedControllerGroup frontRightGroup = new SpeedControllerGroup(frontRight1, frontRight2);
  SpeedControllerGroup frontLeftGroup = new SpeedControllerGroup(frontLeft1, frontLeft2);
  SpeedControllerGroup rearRightGroup = new SpeedControllerGroup(rearRight1, rearRight2);
  SpeedControllerGroup rearLeftGroup = new SpeedControllerGroup(rearLeft1, rearLeft2);

  // Invert the left side motors....if needed.
  // rearLeftGroup.setInverted(true);

  @Override
  public void robotInit() {

    // initialize the drive train
    m_robotDrive = new MecanumDrive(frontLeftGroup, rearLeftGroup, frontRightGroup, rearRightGroup);
    // initialize the joysticks
    xbox_lower = new Joystick(kJoystickChannelLower);
    xbox_upper = new Joystick(kJoystickChannelUpper);

    CameraServer cam0 = CameraServer.getInstance();
    cam0.startAutomaticCapture("cam0", 0);
    CameraServer cam1 = CameraServer.getInstance();
    cam1.startAutomaticCapture("cam1", 1);
    

  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    // m_timer.reset();
    // m_timer.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    try {
      autMove();
    } catch (InterruptedException e) {
       //TODO Auto-generated catch block
      e.printStackTrace();
    }
    
  }
  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    ReadJoystick();
    ReadSensors();
    MoveRobot();
        //////////////////////////////////////////////


  }
  public void ReadJoystick()
  {
    //Xbox upper values 
    x_leftX_upper = xbox_upper.getRawAxis(0);
    x_leftY_upper = xbox_upper.getRawAxis(1);

    x_rightX_upper = xbox_upper.getRawAxis(4);
    x_rightY_upper = xbox_upper.getRawAxis(5);
    x_left_trigger_upper  = xbox_upper.getRawAxis(2);
    x_right_trigger_upper = xbox_upper.getRawAxis(3);
    x_right_bumper_upper  = xbox_upper.getRawButton(6);
    x_left_bumper_upper   = xbox_upper.getRawButton(5);
    x_back_button_upper = (xbox_upper.getRawButton(7));
    x_start_button_upper = (xbox_upper.getRawButton(8));
    x_aButton_upper = (xbox_upper.getRawButton(1));
    x_bButton_upper = (xbox_upper.getRawButton(2));
    x_Xbutton_upper = (xbox_upper.getRawButton(3));
    x_Ybutton_upper = (xbox_upper.getRawButton(4));

    //Xbox lower values 
    x_leftY_lower = xbox_lower.getRawAxis(1);
    x_rightX_lower = xbox_lower.getRawAxis(4);
    x_rightY_lower = xbox_lower.getRawAxis(5);
    x_leftX_lower = xbox_lower.getRawAxis(0);

    x_rightX_lower = xbox_lower.getRawAxis(4);
    x_rightY_lower = xbox_lower.getRawAxis(5);
    x_left_trigger_lower  = xbox_lower.getRawAxis(2);
    x_right_trigger_lower = xbox_lower.getRawAxis(3);
    x_right_bumper_lower  = xbox_lower.getRawButton(6);
    x_left_bumper_lower   = xbox_lower.getRawButton(5);
    x_back_button_lower = (xbox_lower.getRawButton(7));
    x_start_button_lower = (xbox_lower.getRawButton(8));
    x_aButton_lower = (xbox_lower.getRawButton(1));
    x_bButton_lower = (xbox_lower.getRawButton(2));
    x_Xbutton_lower = (xbox_lower.getRawButton(3));
    x_Ybutton_lower = (xbox_lower.getRawButton(4));

    int xPOV_lower =  xbox_lower.getPOV();
  }

  public void MoveRobot()
  {
    double deadband = 0.25;
double exel;
   // boolean rightX_stat, rightY_stat;
      if(x_aButton_lower){
        exel = 1;
      }else {
        exel = 0.5;
      }
          
    // links the triggers
       double RotationControl = (x_right_trigger_lower) + (x_left_trigger_lower * -1.0);
       m_robotDrive.driveCartesian( Math.pow(x_leftX_lower, 3)/2 ,Math.pow(-x_leftY_lower, 3)*exel , Math.pow(RotationControl, 3)/2);

      //accuate climber
      if ( x_back_button_upper && x_aButton_upper){
        S_climb.set(-1.0);
        
      }else{
        S_climb.set(0);
        
      }
      //accuate intake  x_left_trigger_upper (intake)
      // when left trigger pressed intake set to 1. else 0
      if ( x_left_trigger_upper >= deadband){
        M_intake.set(-0.75);
      }else{
        M_intake.set(0);
      }
      // camera movements

      if ( x_Xbutton_upper ) {
        cameraMount.setAngle(0);
      } else if (x_Ybutton_upper ) {
        cameraMount.setAngle(90);
      } else {

      }

       // shooter  x_right_trigger_upper
         
       if ( x_right_trigger_upper >= deadband){
        M_shooter1.set(0.60);
        M_shooter2.set(-0.60);
        M_shooterFeeder.set(0.05);
        ballStop.setAngle(90);
      }else{
        M_shooter1.set(0.0);
        M_shooter2.set(0.0);
        M_shooterFeeder.set(0.0);
        ballStop.setAngle(0);
      }

      // feeder x_right_bumper_upper
      if( x_right_bumper_upper ) {
        M_shooterFeeder.set(-0.2);
      } else {
        M_shooterFeeder.set(0.0);
      }

      //accuate mast      
      if ( x_back_button_upper && (x_leftY_upper >= deadband)){
      
        M_mast.set(-x_leftY_upper);
      } else if( x_back_button_upper && (x_leftY_upper <= -deadband)){
        
        M_mast.set(-x_leftY_upper);
      } else {
        M_mast.set(0.0);
      }
    
      

    //m_robotDrive.driveCartesian( xbox_stick_lower.getX(),  xbox_stick_lower.getY(),
    //xbox_stick_lower.getZ(), 0.0);
  }


  public void ReadSensors()
  {
    //current_angle = gyro_spi.getAccumulatorAverage(); 
    current_angle = gyroSPI.getAngle();
    //System.out.println (current_angle);
    dist.setAutomaticMode(true);
    boolean val = dist.isRangeValid();
    if (val) {
       double range = dist.getRangeMM();
       System.out.print ("*****************\n");
       System.out.print(range);
   
    }
    
       //String s = current_angle+"";
    //SmartDashboard.putString("DB/String 0", s);
    }
  
    public void autMove() throws InterruptedException {
      System.out.println(i);
      M_shooter1.set(0.60);
      M_shooter2.set(-0.60);
      Timer.delay(0.5);
      M_shooterFeeder.set(-0.15);
      Timer.delay(0.5);
      M_shooterFeeder.set(0.0);
      Timer.delay(1);
      M_shooter1.set(0.0);
      M_shooter2.set(0.0);
      int j, k;
      for(i = 0; i < 15; i++)
      {
        if(i < 10){
          m_robotDrive.driveCartesian(0.75, 0.0 ,-0.2);
          Timer.delay(.1);
        } else{
          m_robotDrive.driveCartesian(0.75, 0.0 ,-0.2);
        Timer.delay(.1);
        }
      }
    Timer.delay(20);
     
      //m_robotDrive.driveCartesian(0.0, 0.35 ,0.0);
      //M_intake.set(-0.75);
      //Timer.delay(20);
      //m_robotDrive.driveCartesian(0.0, 0.35 ,0.0);
      //M_intake.set(0);
      //m_robotDrive.driveCartesian( 0.0, 0.0 ,0.0);
  i++;

   /*   while (i < 25){
       
    i++;
    if (i == 1) {
     M_shooter1.set(0.60);
     M_shooter2.set(-0.60);
     M_shooterFeeder.set(-0.15);
     TimeUnit.SECONDS.sleep(5);
    M_shooter1.set(0.0);
    M_shooter2.set(0.0);
    M_shooterFeeder.set(0.0);
    }
    if ( i >= 2 && i <= 7 ) {
      TimeUnit.MILLISECONDS.sleep(1);
    m_robotDrive.driveCartesian(0.0, 0.0 ,-0.5);
    } 
    TimeUnit.MILLISECONDS.sleep(100);
    m_robotDrive.driveCartesian(0.0, 0.35 ,0.0);
    M_intake.set(-0.75);
   // TimeUnit.SECONDS.sleep();
  // m_robotDrive.driveCartesian( 0.0, 0.0 ,0.0);
  
*/
}
 }

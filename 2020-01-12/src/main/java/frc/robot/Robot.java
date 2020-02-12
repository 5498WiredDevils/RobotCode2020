/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
//Gyro
//import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;


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

  private static final int mastExtensionChannel = 7;
  

  private static final int kJoystickChannelUpper = 0;
  private static final int kJoystickChannelLower = 1;

  //Climber variables

  private static final int climb1 = 5;
  private static final int climb2 = 6;

// Shooter
  private static final int shooter1 = 8;
  private static final int shooter2 = 9;

// intake 
  private static final int intake = 10;
  
// mast
  private static final int mast = 7;
// lower chassie drive
  private MecanumDrive m_robotDrive;

  //Gyroscope variables
  Gyro gyroSPI = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  double current_angle;
  double Kp = 0.03;

  //Servo Stuff
  Servo ballStop = new Servo(0);
  Servo cameraMount = new Servo(1);
  
  //Joystick defs
  private Joystick xbox_lower;
  private Joystick xbox_upper;

  //Xbox input variables   
  //upper joystick
  double x_left_trigger_upper, x_right_trigger_upper; // triggers on xbox
  double x_leftY_upper;
  double x_leftX_upper;
  double x_rightY_upper;
  double x_rightX_upper;
  boolean x_right_bumper_upper, x_left_bumper_upper; // bumper on xbox
  boolean x_back_button_upper, x_start_button_upper;
  boolean x_Xbutton_upper, x_Ybutton_upper;
  boolean x_aButton_upper, x_bButton_upper;

//lower joystick
  double x_left_trigger_lower, x_right_trigger_lower; // triggers on xbox
  double x_leftY_lower;
  double x_leftX_lower;
  double x_rightY_lower;
  double x_rightX_lower;
  boolean x_right_bumper_lower, x_left_bumper_lower; // bumper on xbox
  boolean x_back_button_lower, x_start_button_lower;
  boolean x_Xbutton_lower, x_Ybutton_lower;
  boolean x_aButton_lower, x_bButton_lower;
//motors
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
  WPI_TalonSRX M_mast = new WPI_TalonSRX(mast);

  // All Speed Control groups
//climber is a group.
SpeedControllerGroup S_climb = new SpeedControllerGroup(M_climb1, M_climb2);   
//lower chassie groups
SpeedControllerGroup frontRightGroup = new SpeedControllerGroup(frontRight1, frontRight2);
SpeedControllerGroup frontLeftGroup = new SpeedControllerGroup(frontLeft1, frontLeft2);
SpeedControllerGroup rearRightGroup = new SpeedControllerGroup(rearRight1, rearRight2);
SpeedControllerGroup rearLeftGroup = new SpeedControllerGroup(rearLeft1, rearLeft2);

// Invert the left side motors....if needed.
//rearLeftGroup.setInverted(true);


  @Override
  public void robotInit() {
    
    // initialize the drive train
    m_robotDrive = new MecanumDrive(frontLeftGroup, rearLeftGroup, frontRightGroup, rearRightGroup);
    // initialize the joysticks
    xbox_lower = new Joystick(kJoystickChannelLower);
    xbox_upper = new Joystick(kJoystickChannelUpper);
    
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

   // boolean rightX_stat, rightY_stat;

    
    // links the triggers
       double RotationControl = (x_right_trigger_lower * 1.0) + (x_left_trigger_lower * -1.0);
       m_robotDrive.driveCartesian( x_leftX_lower * 0.75 ,-x_leftY_lower , RotationControl );

      //accuate climber
      if ( x_back_button_upper && x_aButton_upper){
        S_climb.set(1.0);
        
      }else{
        S_climb.set(0);
        
      }
      //accuate intake  x_left_trigger_upper (intake)
      // when left trigger pressed intake set to 1. else 0
      if ( x_left_trigger_upper >= deadband){
        M_intake.set(1.0);
      }else{
        M_intake.set(0);
      }
      // camera movements

      if (xbox_lower.getPOV() == 0) {
        cameraMount.set(90);
      } else if (xbox_lower.getPOV() == 180) {
        cameraMount.set(0);
      } else {

      }

       // shooter  x_right_trigger_upper
         
       if ( x_right_trigger_upper >= deadband){
        M_shooter1.set(1.0);
        M_shooter2.set(-1.0);
        ballStop.setAngle(90);
      }else{
        M_shooter1.set(0.0);
        M_shooter2.set(0.0);
        ballStop.setAngle(0);
      }
      //accuate mast      
      if ( x_back_button_upper ){
        M_mast.set(x_leftY_upper);
        
      }else{
        M_mast.set(0);
        
      }
    


    //m_robotDrive.driveCartesian( xbox_stick_lower.getX(),  xbox_stick_lower.getY(),
    //xbox_stick_lower.getZ(), 0.0);
  }


  public void ReadSensors()
  {
    //current_angle = gyro_spi.getAccumulatorAverage(); 
    current_angle = gyroSPI.getAngle();
    //System.out.println (current_angle);
  
    //String s = current_angle+"";
    //SmartDashboard.putString("DB/String 0", s);
  }
  


}

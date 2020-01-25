/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.TimerTask;

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
  private static final int kFrontLeftChannel1 = 1;
  private static final int kFrontLeftChannel2 = 3;

  private static final int kRearLeftChannel1 = 5;
  private static final int kRearLeftChannel2 = 7;
  
  private static final int kFrontRightChannel1 = 2;
  private static final int kFrontRightChannel2 = 4;

  private static final int kRearRightChannel1 = 6;
  private static final int kRearRightChannel2 = 8;



  private static final int kJoystickChannelUpper = 0;
  private static final int kJoystickChannelLower = 1;

  private MecanumDrive m_robotDrive;
  

  //Gyroscope variables
  //private Gyro gyro;
  //SPI gyro_spi;
  
  Gyro gyroSPI = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  //public SPI port;
  //ADXRS450_Gyro 
  double current_angle;
  double Kp = 0.03;
  
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


  @Override
  public void robotInit() {
    WPI_TalonSRX frontLeft1 = new WPI_TalonSRX(kFrontLeftChannel1);
    WPI_TalonSRX rearLeft1 = new WPI_TalonSRX(kRearLeftChannel1);
    WPI_TalonSRX frontRight1 = new WPI_TalonSRX(kFrontRightChannel1);
    WPI_TalonSRX rearRight1 = new WPI_TalonSRX(kRearRightChannel1);

    WPI_TalonSRX frontLeft2 = new WPI_TalonSRX(kFrontLeftChannel2);
    WPI_TalonSRX rearLeft2 = new WPI_TalonSRX(kRearLeftChannel2);
    WPI_TalonSRX frontRight2 = new WPI_TalonSRX(kFrontRightChannel2);
    WPI_TalonSRX rearRight2 = new WPI_TalonSRX(kRearRightChannel2);

    //master slave groups of motors
    SpeedControllerGroup frontRightGroup = new SpeedControllerGroup(frontRight1, frontRight2);
    SpeedControllerGroup frontLeftGroup = new SpeedControllerGroup(frontLeft1, frontLeft2);
    SpeedControllerGroup rearRightGroup = new SpeedControllerGroup(rearRight1, rearRight2);
    SpeedControllerGroup rearLeftGroup = new SpeedControllerGroup(rearLeft1, rearLeft2);
   // WPI_TalonSRX rightFront_master = new WPI_TalonSRX(2);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeftGroup.setInverted(true);
    //rearLeftGroup.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeftGroup, rearLeftGroup, frontRightGroup, rearRightGroup);

    xbox_lower = new Joystick(kJoystickChannelLower);
    xbox_upper = new Joystick(kJoystickChannelUpper);
    // set up  the gyro on channel 1.
    // gyro_spi= new SPI(Port.kOnboardCS0);
    
    // gyro_spi.setMSBFirst();
    // gyro_spi.startT
    //gyro = new AnalogGyro(gyro_spi_port);
   
    

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
  }

  public void MoveRobot()
  {
   // double deadband = 0.25;
   // boolean rightX_stat, rightY_stat;

    

    // if(rightX_stat && rightY_stat){
      //the current angle is the angle that is read from the gyro in the ReadSensors fuction.
      // this will allow you to drive with respect to the user frame.
       //m_robotDrive.driveCartesian(-x_rightY_lower,x_leftX_lower, x_rightX_lower, current_angle);
       //This code will drive with respect to the robot frame.
       m_robotDrive.driveCartesian(-x_rightY_lower,x_leftX_lower, x_rightX_lower );

    



    //m_robotDrive.driveCartesian( xbox_stick_lower.getX(),  xbox_stick_lower.getY(),
    //xbox_stick_lower.getZ(), 0.0);
  }


  public void ReadSensors()
  {
    //current_angle = gyro_spi.getAccumulatorAverage(); 
    current_angle = gyroSPI.getAngle();
    System.out.println (current_angle);
  
    //String s = current_angle+"";
    //SmartDashboard.putString("DB/String 0", s);
  }
  


}

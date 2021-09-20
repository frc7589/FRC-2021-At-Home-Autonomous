/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot;
 
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
 
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.SPI;
 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import java.util.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
//預設程式碼
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
 
//Encoder(int channelA, int channelB, boolean reverseDirection, CounterBase.EncodingType encodingType)
 
// -channel : roborio上的DIO腳位
// -reverseDirection : True:不反轉 flase:要反轉 *可用getDirection()確認
// -EncodingType : 有k1x,k2x,k4x 三種，其差異在於如何計算Encoder轉一圈
  private Encoder m_frontLeftEncoder = new Encoder(0, 1,false,EncodingType.k4X);
  private Encoder m_frontRightEncoder = new Encoder(2, 3,true,EncodingType.k4X);
  private Encoder m_backLeftEncoder = new Encoder(4, 5,false,EncodingType.k4X);
  private Encoder m_backRightEncoder = new Encoder(6, 7,true,EncodingType.k4X);
  
 
//以車子的中心為原點，寫出四個輪子的座標
  private final Translation2d m_frontLeftLocation = new Translation2d(0.295, 0.2538);
  private final Translation2d m_frontRightLocation = new Translation2d(0.295, -0.2538);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.295, 0.2538);
  private final Translation2d m_backRightLocation = new Translation2d(-0.295, -0.2538);
 
//把四個輪子的座標合成一台車
  private final MecanumDriveKinematics m_kinematics =
  new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
 
//Pose2d​(double x, double y, Rotation2d rotation)
 
//-x,y : 車子起始座標
//-rotation(angle) : 設定車子起始角度 
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d(), 
                             new Pose2d(3.81,0.295, new Rotation2d(0)));   //a_blue
  
  private WPI_TalonSRX victorfrontLeft = new WPI_TalonSRX(0);
  private WPI_TalonSRX victorfrontRight = new WPI_TalonSRX(1);
  private WPI_TalonSRX victorbackLeft = new WPI_TalonSRX(2);
  private WPI_TalonSRX victorbackRight = new WPI_TalonSRX(3);
 
  private WPI_VictorSPX ballSucker = new WPI_VictorSPX(4);
//氣動桿的宣告
  private Compressor compressor = new Compressor(35);
  private DoubleSolenoid valve1 = new DoubleSolenoid(35, 1, 2);
 
  
//把四個馬達控制器合成一台車
MecanumDrive robotDrive = new MecanumDrive(victorfrontLeft, victorbackLeft, victorfrontRight, victorbackRight);
 
//設置Navx陀螺儀
  private AHRS ahrs = new AHRS(SPI.Port.kMXP);
 
//4個目標座標及角度
  private Pose2d marker_1 =new Pose2d(3.81,4.572,new Rotation2d(0));
  private Pose2d marker_2 =new Pose2d(1.524,5.334,new Rotation2d(0));
  private Pose2d marker_3 =new Pose2d(2.286,6.858,new Rotation2d(0));
  private Pose2d marker_4 =new Pose2d(2.286,8.382,new Rotation2d(0));
  
//透過NetworkTable接收視覺辨識傳回的值
  private NetworkTableInstance inst;
  private NetworkTable table;
  private NetworkTableEntry x_Entry;
  private NetworkTableEntry y_Entry;
  private NetworkTableEntry z_Entry;
  private NetworkTableEntry sita_Entry;
  private NetworkTableEntry time_Entry;
  private double cord_x;
  private double cord_y;
  private double cord_z;
  private double cord_sita;
  private double cord_time;
 
//變數
  int a = 1;
  int level = 0;
  //private PIDController turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
  }
 
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }
  
 
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
 
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
//encoder轉一圈得到的距離(單位要統一)
    m_frontLeftEncoder.setDistancePerPulse(0.15*3.14/600/1.35);
    m_frontRightEncoder.setDistancePerPulse(0.15*3.14/600/1.35);
    m_backLeftEncoder.setDistancePerPulse(0.15*3.14/600/1.35);
    m_backRightEncoder.setDistancePerPulse(0.15*3.14/600/1.35);
    //m_odometry.resetPosition(new Pose2d(), new Rotation2d());
 
 
//陀螺儀初始化
    ahrs.reset();
//氣動桿程式
    victorfrontRight.setSafetyEnabled(false);
    victorbackRight.setSafetyEnabled(false);
    victorfrontLeft.setSafetyEnabled(false);
    victorbackLeft.setSafetyEnabled(false);
    valve1.set(Value.kForward);
//預設程式碼
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
 
//將相對應的Entry(Key)接收到的值傳給分別的變數
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("SmartDashboard");
    x_Entry = table.getEntry("cord_x");
    y_Entry = table.getEntry("cord_y");
    z_Entry = table.getEntry("cord_z");
    sita_Entry = table.getEntry("cord_sita");
    time_Entry = table.getEntry("cord_time");
  }
 
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
//收球馬達
    ballSucker.set(-.6);
 
//將接收到的變數做處理
    cord_x = x_Entry.getDouble(0);
    cord_y = y_Entry.getDouble(0);
    cord_z = z_Entry.getDouble(0);
    cord_sita = sita_Entry.getDouble(0);
    cord_x = Math.round(cord_x/100)*100;
    cord_y = Math.round(cord_y/100)*100;
    cord_z = Math.round(cord_z/100)*100;
    cord_sita = Math.round(cord_sita/100)*100;
 
 
  Pose2d markerbycar_1 = marker_1.relativeTo(m_odometry.getPoseMeters());
  Pose2d markerbycar_2 = marker_2.relativeTo(m_odometry.getPoseMeters());
  Pose2d carbycar = m_odometry.getPoseMeters().relativeTo(m_odometry.getPoseMeters());
 
//統合4個encoder的速度
  var wheelSpeeds = new MecanumDriveWheelSpeeds(m_frontLeftEncoder.getRate(), m_frontRightEncoder.getRate(),
  m_backLeftEncoder.getRate(), m_backRightEncoder.getRate());
 
//記得要加負號(坐標系轉換)
  var gyro = Rotation2d.fromDegrees(-ahrs.getAngle());
 
//求跟起始角度(0度)的差距角度數
  double cur_deg = ahrs.getAngle()%360; // get the current robot rotation and confine it between [0 ~ 360)
  double target = -90; // set the target degree
  //target = (-target) % 360;
  double delta_deg = target - cur_deg;
  if (delta_deg >= 180) delta_deg -= 360;
  else if (delta_deg < -180) delta_deg += 360; // get the delta degree and confine it to [-180, 180)
 
 
  switch(level){
    case 0:
 
//到每個目標座標都要轉正
//driveCartesian(ySpeed,xSpeed,角速度)
      if(delta_deg>2) {
        robotDrive.driveCartesian(0,0,0.2); 
      }
      else if (delta_deg<-2){
       robotDrive.driveCartesian(0,0,-0.2);
      }
      else{
        robotDrive.driveCartesian(0,0,0);
        level++;
        System.out.println(level);
      }
      break;
 
//經過1個case,到達1個目標座標 
    case 1:
    //System.out.println("hehe");
      Translation2d delta_1 = marker_1.minus(m_odometry.getPoseMeters()).getTranslation();//從目標座標減掉現在的車子座標的變化量取距離
      Translation2d speed_1 =  delta_1.div( delta_1.getNorm() );//取單位向量
      m_odometry.update(gyro, wheelSpeeds);//刷新
      if(delta_1.getNorm()<=0.1){
        robotDrive.driveCartesian(0,0,0);
        level++;
      }
    //距離目標<10公分馬達速度為0
	  //到每個目標座標都要轉正
    //driveCartesian(ySpeed,xSpeed,角速度)
      else if(delta_deg>2) {
        robotDrive.driveCartesian(-speed_1.getY()*0.3,speed_1.getX()*0.3,0.2);
      }
      else if (delta_deg<-2){
       robotDrive.driveCartesian(-speed_1.getY()*0.3,speed_1.getX()*0.3,-0.2);
      }
      else{
        robotDrive.driveCartesian(-speed_1.getY()*0.3,speed_1.getX()*0.3,0);
      }
      
      break;
    
    case 2:
      Translation2d delta_2 = marker_2.minus(m_odometry.getPoseMeters()).getTranslation();
      Translation2d speed_2 =  delta_2.div( delta_2.getNorm() );
      m_odometry.update(gyro, wheelSpeeds);
      System.out.println("odometry");
      System.out.println(m_odometry.getPoseMeters());
      if(delta_2.getNorm()<=0.1){
        robotDrive.driveCartesian(0,0,0);
        level++;
      }
      else if(delta_deg>2) {
        robotDrive.driveCartesian(-speed_2.getY()*0.3,speed_2.getX()*0.3,0.2);
      }
      else if (delta_deg<-2){
       robotDrive.driveCartesian(-speed_2.getY()*0.3,speed_2.getX()*0.3,-0.2);
      }
      else{
        robotDrive.driveCartesian(-speed_2.getY()*0.3,speed_2.getX()*0.3,0);
      }
      
      break;
 
      
      case 3:
      Translation2d delta_3 = marker_3.minus(m_odometry.getPoseMeters()).getTranslation();
      Translation2d speed_3 =  delta_3.div( delta_3.getNorm() );
      m_odometry.update(gyro, wheelSpeeds);
      System.out.println("odometry");
      System.out.println(m_odometry.getPoseMeters());
      if(delta_3.getNorm()<=0.1){
        robotDrive.driveCartesian(0,0,0);
        level++;
      }
      else if(delta_deg>2) {
        robotDrive.driveCartesian(-speed_3.getY()*0.3,speed_3.getX()*0.3,0.2);
      }
      else if (delta_deg<-2){
       robotDrive.driveCartesian(-speed_3.getY()*0.3,speed_3.getX()*0.3,-0.2);
      }
      else{
        robotDrive.driveCartesian(-speed_3.getY()*0.3,speed_3.getX()*0.3,0);
      }
      
      break;
	
      
      case 4:
      Translation2d delta_4 = marker_4.minus(m_odometry.getPoseMeters()).getTranslation();
      Translation2d speed_4 =  delta_4.div( delta_4.getNorm() );
      m_odometry.update(gyro, wheelSpeeds);
      System.out.println("odometry");
      System.out.println(m_odometry.getPoseMeters());
      if(delta_4.getNorm()<=0.1){
        robotDrive.driveCartesian(0,0,0);
        level++;
      }
      else if(delta_deg>2) {
        robotDrive.driveCartesian(-speed_4.getY()*0.3,speed_4.getX()*0.3,0.2);
      }
      else if (delta_deg<-2){
       robotDrive.driveCartesian(-speed_4.getY()*0.3,speed_4.getX()*0.3,-0.2);
      }
      else{
        robotDrive.driveCartesian(-speed_4.getY()*0.3,speed_4.getX()*0.3,0);
      }
      
      break;
 
      }
 
    }
  }

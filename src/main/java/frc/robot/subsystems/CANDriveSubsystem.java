// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging; 


public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final DifferentialDrivePoseEstimator poseEstimator;

  private final RelativeEncoder leftRelativeEncoder;
  private final RelativeEncoder rightRelativeEncoder;

  public DifferentialDriveKinematics kinematics;

  private final AHRS gyro;

  public CANDriveSubsystem() {

    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(200);
    rightLeader.setCANTimeout(200);
    leftFollower.setCANTimeout(200);
    rightFollower.setCANTimeout(200);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig.voltageCompensation(12);
    sparkConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    sparkConfig.follow(leftLeader);
    leftFollower.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkConfig.follow(rightLeader);
    rightFollower.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    sparkConfig.inverted(true);
    sparkConfig.disableFollowerMode();
    rightLeader.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    sparkConfig.inverted(false);
    leftLeader.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftRelativeEncoder = leftLeader.getEncoder();
    rightRelativeEncoder = rightLeader.getEncoder();  

    gyro = new AHRS(NavXComType.kMXP_SPI);

	poseEstimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.kZero, 
		0.0, 0.0, Pose2d.kZero);
    
    odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(),
      leftRelativeEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE,
      rightRelativeEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE
    );

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22.5));
    
    // PathPlanner Stuff
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Blue;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    };
  
  // public void initialize() {
  //   gyro.reset();
  //   resetPose(new Pose2d(getPose().getX() ,getPose().getY(), new Rotation2d()));
  // }

  
  public void periodic() {
    odometry.update(
      gyro.getRotation2d(),
      leftRelativeEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE,
      rightRelativeEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE
    );
    SmartDashboard.putNumber("robotHeading", getHeading());
    SmartDashboard.putNumber("pose rotation - deg", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("pose x" , getPose().getX());
    SmartDashboard.putNumber("pose y" , getPose().getY());
    
    SmartDashboard.putNumber("left encoder pos", leftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("right encoder pos", rightRelativeEncoder.getPosition());
    SmartDashboard.putNumber("current speeds", 
        Math.sqrt(Math.pow(getCurrentSpeeds().vxMetersPerSecond, 2)
         + Math.pow(getCurrentSpeeds().vyMetersPerSecond, 2)));

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      SmartDashboard.putNumber("target pose rotation - deg", pose.getRotation().getDegrees());
    });
    // Logger.recordOutput("MyPoseArray", poseA, poseB);
    // Logger.recordOutput("MyPoseArray", new Pose2d[] {poseA, poseB});
    
  
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    wheelSpeeds.desaturate(0.5);

    drive.tankDrive(-wheelSpeeds.leftMetersPerSecond, -wheelSpeeds.rightMetersPerSecond);
    //drive.tankDrive(-wheelSpeeds.leftMetersPerSecond, -wheelSpeeds.leftMetersPerSecond);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), leftRelativeEncoder.getPosition(), rightRelativeEncoder.getPosition(), pose);
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees() % 360;
  }

  public ChassisSpeeds getCurrentSpeeds() {
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
            	(leftRelativeEncoder.getVelocity() * WHEEL_CIRCUMFERENCE / 60) / WHEEL_GEARING,
				(rightRelativeEncoder.getVelocity() * WHEEL_CIRCUMFERENCE / 60) / WHEEL_GEARING);
    return kinematics.toChassisSpeeds(speeds);
  }

  public void driveStop(){
    drive.arcadeDrive(0, 0);
  }
}

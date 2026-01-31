package frc.robot.commands;

import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class Turn extends Command {
    private final AHRS gyro;

    private final CANDriveSubsystem drive;

    private final double setPoint;

    private final double kP;

    private double error;

    public Turn (AHRS gyro, double setPoint, CANDriveSubsystem drive){
      //  drive.getCurrentSpeeds().fromRobotRelativeSpeeds(chassisSpeeds, null)
      this.gyro = gyro;
      this.setPoint = setPoint;
      this.drive = drive;

      kP = 0.006;
    }
  @Override
  public void initialize() {

  }  

  @Override
  public void execute() {
    error = setPoint - (gyro.getRotation2d().getDegrees() % 360);
    drive.driveArcade(0.0, kP * error);
    SmartDashboard.putNumber("error", error);
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished(){
    if (error < 0.1) {
        return true;
    } else {
        return false;
    }
  }
}

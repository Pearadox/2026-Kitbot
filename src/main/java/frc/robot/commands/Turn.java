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
import frc.robot.util.LoggedTunableNumber;

public class Turn extends Command {

    private final CANDriveSubsystem drive;

    private final double setPoint;

    private final double kP;

    private double error;

    // public final LoggedTunableNumber kP = new LoggedTunableNumber("kP", 0.04);

    public Turn (AHRS gyro, double setPoint, CANDriveSubsystem drive){
      //  drive.getCurrentSpeeds().fromRobotRelativeSpeeds(chassisSpeeds, null)
      this.setPoint = setPoint;
      this.drive = drive;

      kP = 0.0106; //0.04
    }
  @Override
  public void initialize() {
   // drive.resetPose(drive.getPose());

  }  

  @Override
  public void execute() {
    error = setPoint - (drive.getHeading());
    drive.driveArcade(0.0, kP * error);
    SmartDashboard.putNumber("error", error);
  }


  @Override
  public void end(boolean interrupted) {
    drive.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished(){
    if (Math.abs(error) < 15) {
        return true;
    } else {
        return false;
    }
  }
}

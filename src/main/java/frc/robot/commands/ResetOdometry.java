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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetOdometry extends Command {
  /** Creates a new Drive. */
  private final CANDriveSubsystem drive;

  private AHRS gyro;

  public ResetOdometry(AHRS gyro, CANDriveSubsystem drive){
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   drive.resetPose(new Pose2d(drive.getPose().getX() ,drive.getPose().getY(), new Rotation2d()));
   gyro.reset();
  }

  @Override
  public void execute() {
  }


  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}
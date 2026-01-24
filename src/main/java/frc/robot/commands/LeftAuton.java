package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class LeftAuton extends SequentialCommandGroup {
    
        public LeftAuton(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem){
            addCommands(
                //drives forward (considering intake is the front) to prepare to release the kraken
                new AutoDrive(driveSubsystem, 0.5, 0).withTimeout(1),
                //shoots fuel
                new Launch(fuelSubsystem).withTimeout(3),
                //drive to depot
                new AutoDrive(driveSubsystem, 1.0, 0).withTimeout(0.5),
                //intake ground fuel
                new Intake(fuelSubsystem).withTimeout(5),
                //move to shooting position
                new AutoDrive(driveSubsystem, 0.5, 0).withTimeout(1),
                //shoots fuel
                new Launch(fuelSubsystem).withTimeout(5),
                //parks so robot is no in the middle of the field
                new AutoDrive(driveSubsystem, -.5, 0).withTimeout(5)

        );
    }
}
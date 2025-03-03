package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignReef;
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class L4AlignmentSequence extends SequentialCommandGroup{
//give it a strafeoffset for each side in the parameters, when constructing just use the leftreef or rightReef setpoint strafeoffset constants
//this is so we dont need two identical classes
    L4AlignmentSequence(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSubsystem, double strafeOffset){
        var config = new ConfigSystem(Constants.SetpointConstants.Options.l4, coralArm, elevator, algaeArm);
        var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator, algaeArm);
        var configureAlign = new AutoAlignReef(swerveSubsystem, strafeOffset,Constants.SetpointConstants.DistanceOffsets.reefCoralConfigure, 0, 0.04, 0.04);
        var scoreAlign = new AutoAlignReef(swerveSubsystem, strafeOffset, Constants.SetpointConstants.DistanceOffsets.leftReefScore, 0, 0.02, 0.02);
        var scoreCoral = new AutoCoralScore(coralArm);
    }
}

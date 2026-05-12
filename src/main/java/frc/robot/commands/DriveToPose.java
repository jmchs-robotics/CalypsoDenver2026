package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DriveToPose extends Command {

  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");

  private static boolean isFlipped = false;

  private final CommandXboxController driveController;

  private final double threshold = 0.5;

  static {
    thetakP.initDefault(0.01);
    thetakD.initDefault(0.9);
  }

  private final Drive drive;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(thetakP.get(), 0.0, thetakD.get(), new Constraints(8.0, 20.0));

  public DriveToPose(Drive drive, CommandXboxController driveController) {
    this.drive = drive;
    this.driveController = driveController;
    if (drive != null) {
      addRequirements(drive);
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    thetaController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    if (thetakP.hasChanged(hashCode()) || thetakD.hasChanged(hashCode())) {
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
    }

    Pose2d currentPose = drive.getPose();

    double thetaVelocity;
    Rotation2d currentToHubAngle;
    if (!isFlipped) {

      currentToHubAngle =
          Constants.blueHub.getTranslation().minus(currentPose.getTranslation()).getAngle();

      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(), currentToHubAngle.getRadians());
    } else {
      currentToHubAngle =
          Constants.redHub.getTranslation().minus(currentPose.getTranslation()).getAngle();

      thetaVelocity =
          thetaController.calculate(
              drive.getRotation().getRadians(), currentToHubAngle.getRadians());
    }

    if (drive != null
        && Math.abs(drive.getRotation().getDegrees() - currentToHubAngle.getDegrees())
            > threshold) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              -driveController.getLeftY() * drive.getMaxLinearSpeedMetersPerSec(),
              -driveController.getLeftX() * drive.getMaxLinearSpeedMetersPerSec(),
              thetaVelocity, // * drive.getMaxAngularSpeedRadPerSec(),
              drive.getRotation()));
    }
  }
}
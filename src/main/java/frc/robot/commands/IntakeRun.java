package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeRun extends Command {

  private final Intake m_Intake;

  public IntakeRun(Intake Intake) {
    m_Intake = Intake;
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_Intake.setWheelMotor(-0.4);
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.setWheelMotor(0);
  }
}
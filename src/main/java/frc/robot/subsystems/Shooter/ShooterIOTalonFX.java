package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX shooterMotor;
  // private final TalonFX followerMotor;

  public ShooterIOTalonFX() {

    shooterMotor = new TalonFX(9);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.02;
    shooterMotor.getConfigurator().apply(config);

    // followerMotor = new TalonFX(15);
    // followerMotor.setControl(new Follower(9, MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shooterCurrentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.shooterVelocityRotPerSec = shooterMotor.getRotorVelocity().getValueAsDouble();
    inputs.shooterIsConnected = shooterMotor.isConnected();
    // inputs.followerIsConnected = followerMotor.isConnected();
  }

  @Override
  public void setVelocity(double speed) {
    shooterMotor.setControl(new VelocityVoltage(-speed));
  }
}
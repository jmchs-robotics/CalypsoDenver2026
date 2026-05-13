package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {

    public double shooterCurrentAmps = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterVelocityRotPerSec = 0.0;
    public boolean shooterIsConnected = false;
    public boolean followerIsConnected = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVelocity(double speed) {}
}
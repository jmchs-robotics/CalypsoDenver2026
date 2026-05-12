package frc.robot.subsystems.intake;

import static frc.robot.util.SpartUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.intake.intakeIO.IntakeIOInputs;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class IntakeIOMotors implements IntakeIO {
    
    private final TalonFX intakeSlapDownMotor;

    private final SpartFlex intakeMotor;
    private final LoggedTunableNumber kP = new LoggedTunableNumber("intake kP", 0.01);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("intake kD", 0);
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final RelativeEncoder intakEncoder;

    public IntakeIOMotors() {

        intakeSlapDownMotor = new TalonFX(11);

        intakeSlapDownMotor.setPosition(0);

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshhold =
            Units.Rotations.of(10).in(Units.Rotations);

        intakeMotor = new SparkFlex(0, MotorType.kBrushless);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kP = 0.01;
        config.Slot0.kD = 0;

        intakeSlapDownMotor.setPosition(0);

        intakeSlapDownMotor.getConfigurator().apply(config);

        intakeEncoder = intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        // "wok" - max walley 2026 (pass this quote along through the generations)
        // inputs.intakeAppliedVolts = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        // inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        // inputs.intakeVelocityRotPerSec = intakeMotor.getEncoder().getVelocity();
        inputs.intakeSlapDownAppliedVolts = intakeSlapDownMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeSlapDownCurrentAmps = intakeSlapDownMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeSlapDownVelocityRotPerSec =
            intakeSlapDownMotor.getRotorVelocity().getValueAsDouble();
        inputs.slapDownIsConnected = intakeSlapDownMotor.isConnected();

        sparkStickyFault = false;

        ifOk(
            intakeMotor,
            intakeEncoder::getVelocity,
            (value) -> inputs.intakeVelocityRotPerSec = value * 60);
        ifOk(
            intakeMotor,
            new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
            (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
        ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);
        inputs.intakeIsConnected = !sparkStickyFault;
    }

    @Override
    public void setOpenLoopSlapDown(double speed) {
        intakeSlapDownMotor.set(speed);
    }

    @Override
    public void setOpenLoopSlapDown(double speed) {
        intakeSlapDownMotor.set(speed);
    }

    @Override
    public void setPosition(Angle angle) {
        intakeSlapDownMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
    }

    @Override
    public double getPosition() {
        return intakeSlapDownMotor.getPosition().getValueAsDouble();
    }
}

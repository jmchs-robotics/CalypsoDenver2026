package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake {
    
    private final IntakeIO io;

    private final Alert intakeWheelIsAlert = new Alert("intake wheels disconnected", AlertType.kError);
    private final Alert slapDownAlert = new Alert("slap down moter disconnected", AlertType.kError);

    private final IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        intakeWheelIsAlert.set(!inputs.intakeIsConnected);
        slapDownAlert.set(!inputs.slapDownIsConnected);
    }

    public void setWheelMotor(double speed) {
        io.setOpenLoopSlapDown(speed);
    }

    public void setSlapDownMotor(double speed) {
        io.setOpenLoopSlapDown(speed);
    }

    public void setPosition(Angle angle) {
        io.setPosition(angle);
    }

    public double getPosition() {
        return io.getPosition();
    }
}

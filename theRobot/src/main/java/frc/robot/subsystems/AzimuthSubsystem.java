package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AzimuthSubsystem extends SubsystemBase {
    private final TalonFX azimuthFx;

    public AzimuthSubsystem(int azimuthMotorCanID) {
        azimuthFx = new TalonFX(azimuthMotorCanID);

        // TODO: Configure the PID values
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html#position-control
        Slot0Configs slot0Configs = new Slot0Configs();

        azimuthFx.getConfigurator().apply(slot0Configs);
    }

    public void setAngle() {
        // TODO: Set the angle
    }

    public double getAngle() {
        // TODO: Get the angle
        return 0.0;
    }
}

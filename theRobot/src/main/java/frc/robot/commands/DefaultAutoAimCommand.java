package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AzimuthSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultAutoAimCommand extends Command {
    private final AzimuthSubsystem azimuth;
    private final ShooterSubsystem shooter;

    public DefaultAutoAimCommand(AzimuthSubsystem azimuth, ShooterSubsystem shooter) {
        this.azimuth = azimuth;
        this.shooter = shooter;
        addRequirements(azimuth, shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // TODO: Calculate the correct angle to hit our target
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

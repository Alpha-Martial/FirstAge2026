package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AzimuthSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultAutoAimCommand extends Command {
    private final AzimuthSubsystem azimuth;
    private final ShooterSubsystem shooter;

    // TODO: Remove this eventually
    private static final double azimuthDegrees = 45.0;
    private static final double targetRPM = 5000;

    public DefaultAutoAimCommand(AzimuthSubsystem azimuth, ShooterSubsystem shooter) {
        this.azimuth = azimuth;
        this.shooter = shooter;
        addRequirements(azimuth, shooter);
    }

    @Override
    public void initialize() {
        shooter.runRPM(targetRPM);
    }

    @Override
    public void execute() {
        // TODO: Calculate the correct angle to hit our target
        azimuth.setAngleDegrees(azimuthDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

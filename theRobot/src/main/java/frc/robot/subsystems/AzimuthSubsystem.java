package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MotorUtils;
import frc.robot.control.Constants;
import frc.robot.control.HardwareConstants;

public class AzimuthSubsystem extends SubsystemBase {
    private final TalonFX azimuthFx;

    private final MotionMagicVoltage azimuthFxVoltageController = new MotionMagicVoltage(0);

    private boolean shooterIsAtDesiredAngle = true; // don't start moving until angle is set.
    private double desiredAngleDegrees;
    private double internalAngleOffsetDegrees = 0;
    private static final double shooterAngleLowVelocityTol = 10;

    public AzimuthSubsystem(int azimuthMotorCanID) {
        azimuthFx = new TalonFX(azimuthMotorCanID);

        // TODO: Double check this config
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = HardwareConstants.ctreStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = HardwareConstants.ctreSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.SupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;

        config.MotionMagic.MotionMagicCruiseVelocity = 800.0;
        config.MotionMagic.MotionMagicAcceleration = 160;
        config.MotionMagic.MotionMagicJerk = 800;

        // TODO: Configure the PID values
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/basic-pid-control.html#position-control
        Slot0Configs slot0Configs = new Slot0Configs();

        config.Slot0 = slot0Configs;

        StatusCode response = azimuthFx.getConfigurator().apply(config);
        if (!response.isOK()) {
            DataLogManager.log(
                    "TalonFX ID " + azimuthFx.getDeviceID() + " failed config with error " + response.toString());
        }
        setInternalEncoderOffset();
    }

    /**
     * A method to get the shooter angle
     * 
     * @return angle in degrees
     */
    public double getAngleDegrees() {
        return rotationsToDegrees(azimuthFx.getPosition().getValueAsDouble()) + internalAngleOffsetDegrees;
    }

    /**
     * A method to test whether the angle is within tolerance of the target angle
     * 
     * @param targetAngleDegrees
     * @return true if the angle is within tolerance
     */
    public boolean isAngleWithinTolerance(double targetAngleDegrees) {
        // check both the position and velocity. To allow PID to not stop before
        // settling.
        boolean positionTargetReached = Math
                .abs(getAngleDegrees() - targetAngleDegrees) < Constants.shooterAngleToleranceDegrees;
        boolean velocityIsSmall = Math.abs(azimuthFx.getVelocity().getValueAsDouble()) < shooterAngleLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        if (!shooterIsAtDesiredAngle) {
            // use motionMagic voltage control
            azimuthFx.setControl(
                    azimuthFxVoltageController
                            .withPosition(degreesToRotations(desiredAngleDegrees - internalAngleOffsetDegrees)));
            // angleRightMotor acts as a follower
            // keep moving until it reaches target angle
            shooterIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
        }

        SmartDashboard.putNumber("Shooter Motor Encoder Degrees", getAngleDegrees());
        SmartDashboard.putNumber("Shooter Angle Motor Rotations ", azimuthFx.getPosition().getValueAsDouble());
    }

    /**
     * A method to set the shooter angle
     * 
     * @param degrees
     */
    public void setAngleDegrees(double degrees) {
        double clampedDegrees = MotorUtils.clamp(degrees, Constants.shooterAngleMinDegrees,
                Constants.shooterAngleMaxDegrees);
        if (clampedDegrees != degrees) {
            DataLogManager.log("Warning: Shooter Angle requested degrees of " + degrees +
                    "exceeded bounds of [" + Constants.shooterAngleMinDegrees + " .. "
                    + Constants.shooterAngleMaxDegrees +
                    "]. Clamped to " + clampedDegrees + ".");
        }
        desiredAngleDegrees = clampedDegrees;
        shooterIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
    }

    public double getAngle() {
        return rotationsToDegrees(azimuthFx.getPosition().getValueAsDouble()) + internalAngleOffsetDegrees;
    }

    private void setInternalEncoderOffset() {
        // only call this at startup!
        internalAngleOffsetDegrees = Constants.shooterStartingAngleOffsetDegrees
                - rotationsToDegrees(azimuthFx.getPosition().getValueAsDouble());
    }

    private double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    private double rotationsToDegrees(double rotations) {
        return rotations * 360;
    }
}

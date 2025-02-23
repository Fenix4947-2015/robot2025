package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Balls extends SubsystemBase {

    private final SparkMax m_motor = new SparkMax(ElectricConstants.kBallsCanId, SparkLowLevel.MotorType.kBrushless);

    public static double DEFAULT_SPEED = 0.5;

    public Balls() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void roll(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        m_motor.set(0.0);
    }

    @Override
    public void periodic() {
    }
}

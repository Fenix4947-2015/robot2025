package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class CageGripper extends SubsystemBase {

    private final SparkMax m_motor = new SparkMax(ElectricConstants.kWinchCageGripperChannel, SparkLowLevel.MotorType.kBrushless);

    public CageGripper() {
        SparkMaxConfig cageGripperConfig = new SparkMaxConfig();
        cageGripperConfig.idleMode(IdleMode.kBrake);

        m_motor.configure(cageGripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void roll(double speed) {
        m_motor.set(speed);
    }


    @Override
    public void periodic() {
    }

}

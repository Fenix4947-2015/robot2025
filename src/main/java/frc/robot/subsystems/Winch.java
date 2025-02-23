package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;
import frc.robot.SmartDashboardWrapper;

public class Winch extends SubsystemBase {

    private final SparkMax m_motorOne = new SparkMax(ElectricConstants.kWinchMotorOneChannel, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motorTwo = new SparkMax(ElectricConstants.kWinchMotorTwoChannel, SparkLowLevel.MotorType.kBrushless);

    private final Encoder m_encoder = new Encoder(ElectricConstants.kWinchEncoderChannel1, ElectricConstants.kWinchEncoderChannel2);

    //private final DigitalInput m_safetySwitch = new DigitalInput(ElectricConstants.kWinchSafetySwitchChannel);

    private static final boolean PREVENT_UNROLL = false;

    public Winch() {
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kBrake);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kBrake).follow(m_motorOne);

        m_motorOne.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motorTwo.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void roll(double speed) {
        if (PREVENT_UNROLL) {
            speed = Math.min(speed, 0.0);
        }
        //if (m_safetySwitch.get()) {
        //    speed = Math.max(speed, 0.0);
        //}

        m_motorOne.set(speed);
    }

    public double getEncoderDistance() {
        return m_encoder.getDistance();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboardWrapper.putNumber("Winch / Distance", getEncoderDistance());
        //SmartDashboardWrapper.putBoolean("Winch / safetySwitch", m_safetySwitch.get());
    }

}

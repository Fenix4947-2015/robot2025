package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SmartDashboardWrapper;

public class Arm extends SubsystemBase {

    private final SparkMax m_motor1 = new SparkMax(Constants.ElectricConstants.kArmMotor1CanId, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(Constants.ElectricConstants.kArmMotor2CanId, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motor3 = new SparkMax(Constants.ElectricConstants.kArmMotor3CanId, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motor4 = new SparkMax(Constants.ElectricConstants.kArmMotor4CanId, SparkLowLevel.MotorType.kBrushless);

    private final CANcoder m_encoder = new CANcoder(Constants.ElectricConstants.kArmCancoderCanId);;

    private final DigitalInput m_lowLimitSwitch = new DigitalInput(Constants.ElectricConstants.kArmLowLimitSwitchChannel);

    private final PIDController m_pidController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Arm.kS, Constants.Arm.kV);
    private double directOutput = 0;
    private ArmMode armMode = ArmMode.DIRECT;

    private enum ArmMode {
        DIRECT,
        PID
    }

    public Arm() {
        //resetEncoder();

        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kBrake);
        m_motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(IdleMode.kBrake).follow(m_motor1.getDeviceId());
        m_motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config3 = new SparkMaxConfig();
        config3.idleMode(IdleMode.kBrake).follow(m_motor1.getDeviceId());
        m_motor3.configure(config3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config4 = new SparkMaxConfig();
        config4.idleMode(IdleMode.kBrake).follow(m_motor1.getDeviceId());
        m_motor4.configure(config4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_pidController.setTolerance(Constants.Arm.kToleranceDegrees);
    }

    public void setTargetPosition(double position) {
        // Clamp the position to be within the physical limits of the arm
        position = Math.min(position, Constants.Arm.kLowestPosition);
        position = Math.max(position, Constants.Arm.kHighestPosition);
        m_pidController.setSetpoint(position);
    }

    public boolean atSetpoint() {
        return m_pidController.atSetpoint();
    }

    public void setDirectOutput(double directOutput) {
        this.directOutput = directOutput;
    }

    public void setPidMode() {
        this.armMode = ArmMode.PID;
    }

    public void setDirectMode() {
        this.armMode = ArmMode.DIRECT;
    }

    public double getEncoderDistance() {
        return -m_encoder.getPosition().getValueAsDouble();
    }

    public void resetEncoder() {
        // TODO: find how to reset encoder in wpilib 2025
        //m_encoder.reset();
    }

    private void movePid() {
        double rawOutput = m_pidController.calculate(getEncoderDistance()) + feedForward.calculate(0);
        double output = limitOutput(rawOutput, getEncoderDistance());
        log(output);
        m_motor1.set(output);
    }

    private void moveDirect() {
        double output = limitOutput(this.directOutput, getEncoderDistance());
        log(output);
        m_motor1.set(output);
    }

    @Override
    public void periodic() {
        if (armMode == ArmMode.PID) {
            movePid();
        } else {
            moveDirect();
        }
    }

    private void log(double output) {
        SmartDashboardWrapper.putNumber("Arm / Output", output);
        SmartDashboardWrapper.putNumberImportant("Arm / Distance", getEncoderDistance());
        SmartDashboardWrapper.putBoolean("Arm / Low limit switch", m_lowLimitSwitch.get());
    }

    private double limitOutput(double output, double position) {
        if (position < Constants.Arm.kLowestPosition || m_lowLimitSwitch.get()) {
            return Math.max(output, 0);
        }
        if (position > Constants.Arm.kHighestPosition) {
            return Math.min(output, 0);
        }
        return output;
    }
}

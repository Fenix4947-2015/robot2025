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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectricConstants;
import frc.robot.SmartDashboardWrapper;

public class Arm extends SubsystemBase {

    private final CoralGripper m_coralGripper;

    private final SparkMax m_motor1 = new SparkMax(ElectricConstants.kArmMotor1CanId, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(ElectricConstants.kArmMotor2CanId, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax m_motor3 = new SparkMax(ElectricConstants.kArmMotor3CanId, SparkLowLevel.MotorType.kBrushless);

    private final CANcoder m_encoder = new CANcoder(ElectricConstants.kArmCancoderCanId);

    private final DigitalInput m_lowLimitSwitch = new DigitalInput(ElectricConstants.kArmLowLimitSwitchChannel);

    private final Solenoid m_extender = new Solenoid(ElectricConstants.kPneumaticHubCanId, PneumaticsModuleType.REVPH, ElectricConstants.kArmExtenderChannel);

    private final PIDController m_pidController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Arm.kS, Constants.Arm.kV);
    private double directOutput = 0;
    private ArmMode armMode = ArmMode.DIRECT;

    public enum ArmMode {
        DIRECT,
        PID
    }

    private DropPosition currDropPosition = null;
    private boolean armRetracted = false;

    public enum DropPosition {
        L4(Constants.Arm.kCoralL4Position, false, false),
        L3(Constants.Arm.kCoralL3Position, true, false),
        L2(Constants.Arm.kCoralL2Position, true, false),
        LOW(Constants.Arm.kLowestPosition, false, true);

        private final double distance;
        private final boolean retracted;
        private final boolean gripperMustBeOpen;

        DropPosition(double distance, boolean retracted, boolean gripperMustBeOpen) {
            this.distance = distance;
            this.retracted = retracted;
            this.gripperMustBeOpen = gripperMustBeOpen;
        }
    }

    public Arm(CoralGripper coralGripper) {
        m_coralGripper = coralGripper;

        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kBrake);
        m_motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(IdleMode.kBrake).follow(m_motor1.getDeviceId());
        m_motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config3 = new SparkMaxConfig();
        config3.idleMode(IdleMode.kBrake).follow(m_motor1.getDeviceId());
        m_motor3.configure(config3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_pidController.setTolerance(Constants.Arm.kToleranceDistance);
        m_pidController.setSetpoint(getEncoderDistance());
    }

    public void setTargetPosition(double position) {
        // Clamp the position to be within the physical limits of the arm
        position = Math.max(position, Constants.Arm.kLowestPosition);
        position = Math.min(position, Constants.Arm.kHighestPosition);
        m_pidController.setSetpoint(position);
    }

    public void setTargetPositionAsCurrent() {
        setTargetPosition(getEncoderDistance());
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

    public ArmMode getArmMode() {
        return armMode;
    }

    public void setNoDropPosition() {
        currDropPosition = null;
    }

    public DropPosition getCurrDropPosition() {
        return currDropPosition;
    }

    public void goToDropPosition(DropPosition dropPosition) {
        this.currDropPosition = dropPosition;
        setPidMode();
        setTargetPosition(this.currDropPosition.distance);
        armRetracted = this.currDropPosition.retracted;
    }

    public double getEncoderDistance() {
        return -m_encoder.getPosition().getValueAsDouble();
    }

    private void movePid() {
        double rawOutput = m_pidController.calculate(getEncoderDistance());
        // disable feedforward when arm is at lowest position
        double ffOutput = rawOutput + (armAtLimitOutputUntilPosition() ? 0.0 : feedForward.calculate(rawOutput));
        double output = limitOutput(ffOutput);
        log(output);
        m_motor1.set(output);
    }

    private void moveDirect() {
        double output = limitOutput(this.directOutput);
        log(output);
        m_motor1.set(output);
    }

    public void moveExtenderIfRequired() {
        //System.out.println("m_extender.get:" + m_extender.get());
        //System.out.println("armRetracted:" + armRetracted);
        if (armRetracted && getEncoderDistance() < Constants.Arm.lowestRetractedPosition) {
            m_extender.set(false);
        } else {
            m_extender.set(armRetracted);
        }
    }

    @Override
    public void periodic() {
        if (armMode == ArmMode.PID) {
            movePid();
        } else {
            moveDirect();
        }

        moveExtenderIfRequired();
    }

    private void log(double output) {
        SmartDashboardWrapper.putNumber("Arm / Output", output);
        SmartDashboardWrapper.putNumberImportant("Arm / Distance", getEncoderDistance());
        SmartDashboardWrapper.putNumber("Arm / Setpoint", m_pidController.getSetpoint());
        SmartDashboardWrapper.putBoolean("Arm / Low limit switch", lowLimitSwitchPushed());
        SmartDashboardWrapper.putBoolean("Arm / At setpoint", atSetpoint());
    }

    private boolean lowLimitSwitchPushed() {
        return !m_lowLimitSwitch.get();
    }

    private boolean armAtLowestPos() {
        return getEncoderDistance() < Constants.Arm.kLowestPosition || lowLimitSwitchPushed();
    }

    private boolean armAtLimitOutputUntilPosition() {
        return getEncoderDistance() < Constants.Arm.kLimitOutputUntilPosition || lowLimitSwitchPushed();
    }

    private boolean armAtHighestPos() {
        return getEncoderDistance() > Constants.Arm.kHighestPosition;
    }

    private double limitOutput(double output) {
        output = limitOutputWhenLow(output);
        if (armAtLowestPos() || isInGripperDangerZone()) {
            return Math.max(output, 0);
        }
        if (armAtHighestPos()) {
            return Math.min(output, 0);
        }
        return output;
    }

    private double limitOutputWhenLow(double output) {
        if (getEncoderDistance() < Constants.Arm.kLimitOutputUntilPosition) {
            return output > 0 ? Math.min(output, 0.5) : Math.max(output, -0.5);
        }
        return output;
    }

    private boolean isInGripperDangerZone() {
        return !m_coralGripper.isSideGripperOpen() && getEncoderDistance() < Constants.Arm.gripperDangerZonePosition;
    }

    public void extendExtender() {
        armRetracted = true;
    }

    public void retractExtender() {
        armRetracted = false;
    }

    public void toggleExtender() {
        armRetracted = !armRetracted;
    }

    public boolean isArmRetracted() {
        return armRetracted;
    }
}

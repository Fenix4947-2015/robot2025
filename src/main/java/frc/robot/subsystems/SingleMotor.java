package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotor extends SubsystemBase{
    private final TalonFX motor;

    public SingleMotor(int deviceId) {
        motor = new TalonFX(deviceId, "CANivore");
    }

    public void run(double speed) {
        motor.set(speed);
    }
}

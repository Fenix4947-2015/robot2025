package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardSettings {

    private double _pidP;
    private double _pidI;
    private double _pidD;
    private double _pidF;
    //private double _kMotor = (int) Launcher.FAR_DOWN_WHEEL_SPEED * 100;
    private String _pidType = "LAUNCHERDOWN";

    public SmartDashboardSettings() {
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("pidP", _pidP);
        SmartDashboard.putNumber("pidI", _pidI);
        SmartDashboard.putNumber("pidD", _pidD);
        SmartDashboard.putNumber("pidF", _pidF);
        //SmartDashboard.putNumber("kMotor", _kMotor);
        SmartDashboard.putString("pidType", _pidType);
    }

    public void refreshPidValues() {
        _pidP = SmartDashboard.getNumber("pidP", _pidP);
        _pidI = SmartDashboard.getNumber("pidI", _pidI);
        _pidD = SmartDashboard.getNumber("pidD", _pidD);
        _pidF = SmartDashboard.getNumber("pidF", _pidF);
        //_kMotor = SmartDashboard.getNumber("kMotor", (double) Launcher.FAR_DOWN_WHEEL_SPEED * 100.0);
        _pidType = SmartDashboard.getString("pidType", _pidType);
    }

    public void setPidValues(double pidP, double pidI, double pidD, double pidF, String pidType) {
        _pidP = pidP;
        _pidI = pidI;
        _pidD = pidD;
        _pidF = pidF;
        _pidType = pidType;
        initSmartDashboard();
    }

    public String getPidType() {
        return _pidType;
    }

    public double getPidP() {
        return _pidP;
    }

    public double getPidI() {
        return _pidI;
    }

    public double getPidD() {
        return _pidD;
    }

    public double getPidF() {
        return _pidF;
    }

//  public double getkMotor() {
//    return _kMotor;
//  }

}

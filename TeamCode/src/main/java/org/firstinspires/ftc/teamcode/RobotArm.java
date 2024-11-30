package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotArm {
    public DcMotor armMotor;
    public CRServo servoMotor;
    private final double     COUNTS_PER_MOTOR_REV    = 28 ;   // rev motorun 1 tur dönüşte enkoder sayımı
    private final double     DRIVE_GEAR_REDUCTION    = 125.0 ;     // 5*5*5 = 125 dişli oranı
    private final double     COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION) / 360 ;
    // 1 tur -> 1680 count
    // 1 derece -> 1680/360 ~= 4.66

    RobotArm(DcMotor armMotor, CRServo servoMotor) {
        this.armMotor = armMotor;
        this.servoMotor = servoMotor;
    }

    public void initArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(armMotor.getCurrentPosition()); // motora şimdiki konumunu korumasını söylüyoruz.
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0);
    }

    public void setArmPositionDegrees(int degrees) {
        armMotor.setTargetPosition((int)(COUNTS_PER_DEGREE * degrees));
        armMotor.setPower(0.3);
    }
    public int getArmPositionDegrees() {
        return (int)(armMotor.getCurrentPosition()/COUNTS_PER_DEGREE);
    }
    public void waitArm() {
        while(armMotor.isBusy());
    }
}

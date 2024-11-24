package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // gyro konumlanmasını ayarlamak için gerekli class
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // otonom
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor; // dc motor classı
import com.qualcomm.robotcore.hardware.IMU; // imu classı (gyro için)
import com.qualcomm.robotcore.util.ElapsedTime; // geçen zaman ölçümü için gerekli class
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "IZFEN OTONOM")
public class FTC_AUTO extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(); // Ne kadar zaman geçtiğini ölçen değişken
    private DcMotor leftDrive = null; // sol motor
    private DcMotor rightDrive = null; // sağ motor
    private IMU imu = null; // inertial measurement unit robotun dönme açısını ölçmek için kullanılıyor

    private double          headingError  = 0; // hedef -> 45 heading -> 30 --> hedef - heading = 15 derece

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0; // hedef (mesela 45 derece)
    private double  driveSpeed    = 0; // sürüş hızı
    private double  turnSpeed     = 0; // dönüş hızı
    private double  leftSpeed     = 0; // sol teker hızı
    private double  rightSpeed    = 0; // sağ teker hızı
    private int     leftTarget    = 0; // sol tekerin enkoder hedefi
    private int     rightTarget   = 0; // sağ tekerin enkoder hedefi


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // rev motorun 1 tur dönüşte enkoder sayımı
    static final double     DRIVE_GEAR_REDUCTION    = 60.0 ;     // 3*4*5 = 60 dişli oranı
    static final double     WHEEL_DIAMETER_CENTIMETER   = 4.0 * 2.54;     // santimetre cinsinden teker çapı
    static final double     COUNTS_PER_CENTIMETER         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETER * 3.1415); // 1 cm'ye düşen enkoder  count sayısı


    static final double     DRIVE_SPEED             = 1;     // maksimum sürüş hızı
    static final double     TURN_SPEED              = 0.6;     // maksimum dönüş hızı
    static final double     HEADING_THRESHOLD       = 1.0 ;    // açı hata payı (45 derece hedef ise 44 ve 46 arasındaki açılar kabul edilir)

    static final double     P_TURN_GAIN            = 0.02;     // bu değişken ile dönülmesi gereken açı çarpılır ve dönüş hızı elde edilir
    static final double     P_DRIVE_GAIN           = 0.03;     // bu değişken ile gidilmesi gereken mesafe (error) çarpılır ve gidiş hızı elde edilir



    @Override
    public void runOpMode() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; // rev logosu yukarı bakıyor
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD; // usb portları geriye bakıyor
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu"); // IMU (gyro)
        imu.initialize(new IMU.Parameters(orientationOnRobot)); // gyro'yu başlat


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive"); // sol motoru tanımla
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive"); // sağ motoru tanımla

        // durdur ve enkoderi sıfırla
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // motorları fren moduna al
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // motorları enkoderli çalışma moduna al
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // yaw eksenini sıfırla (YAW PITCH ROLL)
        imu.resetYaw();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        // düz gidiş için bir tarafı ters çeviriyoruz

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        driveStraight(DRIVE_SPEED, 245,getHeading()); // 245cm düz git
        turnToHeading( TURN_SPEED, getHeading() - 90.0); // saat yönünde 90 derece dön
        driveStraight(DRIVE_SPEED,190,getHeading()); // 190cm düz git
        turnToHeading( TURN_SPEED, getHeading() - 180.0); // 180 derece saat yönünde dön
        driveStraight(DRIVE_SPEED,190,getHeading()); // 190cm düz git
        turnToHeading( TURN_SPEED, getHeading() + 90.0); // 90 derece saat yönünün tersine dön
        driveStraight(DRIVE_SPEED,245,getHeading()); // 245cm düz git (başladığı konuma yakın bir yere geri döner)
        while (opModeIsActive()) {

            sendTelemetry(true); // değişkenleri vs. driver stationa gönder
            // telemetry.update();
        }
    }
    // düz (açıyı koruyarak, dış etkenlere karşı dayanıklı) sürüş
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_CENTIMETER); // enkoder kaç count saymalı hesaplıyoruz
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    // açıya dönüş için robotu hareket ettir.
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // istenen açıya dönüş için gereken motor hızlarını hesapla
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    // robotu hareket ettir (gidiş hızı, dönüş hızı)
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Hız +1 ve -1 arasında değilse hızı bölerek azalt
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }
    // robot açısını al
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}
#include "NRL.h"

// Confirmed wiring: Motor 1 drives both left wheels; Motor 2 drives both right.
// Match the official SDK differential-drive example's mirrored right motor.
static HexaDCMotor leftMotor{{.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM}};
static HexaDCMotor rightMotor{{.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true}};
static TankDrive drive(leftMotor, rightMotor);

class HyperdriveTeleOp : public NRLOpMode {
public:
    void init() override {
        _ready = leftMotor.begin() && rightMotor.begin();
        drive.setRampRate(0.05f);   // Smooth acceleration and reversals.
        drive.stop();
    }

    void loop() override {
        if (!_ready || NRLComms::isInputStale()) {
            drive.stop();
            telemetry.addData("drive", _ready ? "WAITING FOR CONTROLLER" : "MOTOR INIT FAILED");
            return;
        }

        // Left stick Y controls forward/reverse; right stick X steers.
        drive.drive(gamepad1.leftY(), gamepad1.rightX());
        telemetry.addData("drive", "READY");
        telemetry.addData("forward", gamepad1.leftY());
        telemetry.addData("turn", gamepad1.rightX());
        telemetry.addData("speed", "FULL RANGE");
    }

    void stop() override {
        drive.stop();
        HexaDCMotor::stopAll();
    }

private:
    bool _ready = false;
};

REGISTER_OPMODE(HyperdriveTeleOp, "HyperdriveTeleOp", TELEOP);

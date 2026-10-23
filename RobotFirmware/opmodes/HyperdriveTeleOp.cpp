#include "NRL.h"

// Confirmed wiring: Motor 1 drives both left wheels; Motor 2 drives both right.
static HexaDCMotor leftMotor{{.dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM}};
static HexaDCMotor rightMotor{{.dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true}};
static TankDrive drive(leftMotor, rightMotor);

namespace {
// Servo output is enabled only after a deliberate 1 s LB+RB hold.
// These are initial test limits, not guaranteed mechanical endpoints. Adjust
// after testing the mounted linkage with the robot supported and unloaded.
// Extended arm command range. The physical lower endpoint still needs to be
// confirmed on the assembled robot; stop if the linkage binds or the servo strains.
constexpr float kArmMinDeg = 20.0f;
constexpr float kArmMaxDeg = 160.0f;
constexpr float kGripperMinDeg = 60.0f;
constexpr float kGripperMaxDeg = 120.0f;
constexpr float kInitialArmDeg = 90.0f;
constexpr float kInitialGripperDeg = 90.0f;
constexpr float kServoRateDegPerSec = 90.0f;
constexpr uint32_t kEnableHoldMs = 1000;
constexpr uint32_t kTelemetryMs = 200;
}

class HyperdriveTeleOp : public NRLOpMode {
public:
    void init() override {
        _ready = leftMotor.begin() && rightMotor.begin();
        drive.setRampRate(0.05f);
        drive.stop();
        // Defer servo begin() until the operator deliberately enables them.
        telemetry.addData("servos", "HOLD LB+RB 1s TO ENABLE");
    }

    void loop() override {
        const uint32_t now = millis();
        if (!_ready || NRLComms::isInputStale()) {
            drive.stop();
            // Loss of controller link also releases servos and latches them
            // off; restart the OpMode to explicitly re-enable.
            if (NRLComms::isInputStale()) detachAndLatch("LINK LOST");
            publishTelemetry(now);
            return;
        }

        // This controller reports negative leftY when pushed forward.
        drive.drive(-gamepad1.leftY(), -gamepad1.rightX());

        const bool armSelect = gamepad1.pressed(BTN_X);
        const bool gripperSelect = gamepad1.pressed(BTN_A);
        const bool up = gamepad1.pressed(BTN_DPAD_UP);
        const bool down = gamepad1.pressed(BTN_DPAD_DOWN);
        const bool lb = gamepad1.pressed(BTN_LB);
        const bool rb = gamepad1.pressed(BTN_RB);
        const bool idle = !(armSelect || gripperSelect || up || down || lb || rb);

        if (gamepad1.pressed(BTN_Y) || gamepad1.pressed(BTN_LT)) {
            detachAndLatch("SERVO OFF");
        }

        if (!_latched) {
            if (_releaseRequired) {
                if (idle) _releaseRequired = false;
            } else if (lb || rb) {
                // Both bumpers alone for one second enable both servos at the
                // declared starting angles. Releasing/adding another control
                // cancels the timer.
                if (lb && rb && !armSelect && !gripperSelect && !up && !down) {
                    if (!_holdingEnable) {
                        _holdingEnable = true;
                        _enableSince = now;
                    } else if (static_cast<uint32_t>(now - _enableSince) >= kEnableHoldMs) {
                        enableServos();
                        _holdingEnable = false;
                        _releaseRequired = true;
                    }
                } else {
                    _holdingEnable = false;
                }
            } else {
                _holdingEnable = false;
                // Move only while the matching face button and D-pad direction
                // are both held; driving remains available at the same time.
                if (_enabled && up != down) {
                    if (armSelect && !gripperSelect) move(_arm, up ? 1.0f : -1.0f,
                                                          kArmMinDeg, kArmMaxDeg);
                    if (gripperSelect && !armSelect) move(_gripper, up ? 1.0f : -1.0f,
                                                          kGripperMinDeg, kGripperMaxDeg);
                }
            }
        }

        publishTelemetry(now);
    }

    void stop() override {
        drive.stop();
        detachAndLatch("STOPPED");
        HexaDCMotor::stopAll();
    }

private:
    HexaServo _arm{{.signalPin = SERVO_2, .minAngle = kArmMinDeg,
                    .maxAngle = kArmMaxDeg, .startAngle = kInitialArmDeg,
                    .settleMs = 0}};
    HexaServo _gripper{{.signalPin = SERVO_3, .minAngle = kGripperMinDeg,
                        .maxAngle = kGripperMaxDeg, .startAngle = kInitialGripperDeg,
                        .settleMs = 0}};
    bool _ready = false;
    bool _enabled = false;
    bool _latched = false;
    bool _releaseRequired = true;
    bool _holdingEnable = false;
    uint32_t _enableSince = 0;
    uint32_t _lastTelemetry = 0;
    const char* _stopReason = "STOPPED";

    void enableServos() {
        _arm.setRampRate(kServoRateDegPerSec);
        _gripper.setRampRate(kServoRateDegPerSec);
        if (!_arm.begin() || !_gripper.begin()) {
            detachAndLatch("SERVO INIT FAILED");
            return;
        }
        _enabled = true;
    }

    void move(HexaServo& servo, float direction, float minDeg, float maxDeg) {
        const float next = servo.getPosition() + direction *
                           kServoRateDegPerSec * 0.01f; // held input, 100 Hz loop
        if (next < minDeg) servo.setPosition(minDeg);
        else if (next > maxDeg) servo.setPosition(maxDeg);
        else servo.setPosition(next);
    }

    void detachAndLatch(const char* reason) {
        if (_latched) return;
        _arm.detach();
        _gripper.detach();
        _enabled = false;
        _holdingEnable = false;
        _latched = true;
        _stopReason = reason;
    }

    void publishTelemetry(uint32_t now) {
        if (static_cast<uint32_t>(now - _lastTelemetry) < kTelemetryMs) return;
        _lastTelemetry = now;
        telemetry.addData("drive", _ready ? "READY" : "MOTOR INIT FAILED");
        telemetry.addData("forward", -gamepad1.leftY());
        telemetry.addData("turn", -gamepad1.rightX());
        telemetry.addData("speed", "FULL RANGE");
        telemetry.addData("servos", _latched ? _stopReason :
                          (_enabled ? "ENABLED; X/A + UP/DOWN" :
                           (_holdingEnable ? "HOLD LB+RB" : "HOLD LB+RB 1s TO ENABLE")));
        telemetry.addData("arm deg", _arm.isReady() ? _arm.getPosition() : -1.0f);
        telemetry.addData("gripper deg", _gripper.isReady() ? _gripper.getPosition() : -1.0f);
        telemetry.addData("servo limits", "verify mechanism endpoints");
    }
};

REGISTER_OPMODE(HyperdriveTeleOp, "HyperdriveTeleOp", TELEOP);

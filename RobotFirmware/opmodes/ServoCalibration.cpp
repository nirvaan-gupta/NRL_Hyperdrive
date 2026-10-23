#include "NRL.h"
#include <optional>

// Calibrate one channel at a time without commanding drive motors.
// X selects S1/J18 (arm); A selects S3/J17 (gripper).
// Hold LB+RB alone for 1 second to enable at 90 degrees. Then hold D-pad
// UP/DOWN to move continuously at a slow test rate. Y/LT/link loss detaches.
// The initial angle and 60..120 degree range are cautious test settings, not
// known safe endpoints. Verify the servo wiring and voltage before enabling.

namespace {
constexpr float kTrialAngle = 90.0f;
constexpr float kArmMinAngle = 20.0f;
constexpr float kArmMaxAngle = 180.0f;
constexpr float kGripperMinAngle = 60.0f;
constexpr float kGripperMaxAngle = 120.0f;
constexpr float kRateDegPerSec = 30.0f;
constexpr uint32_t kEnableHoldMs = 1000;
constexpr uint32_t kTelemetryMs = 200;

struct CalibrationServo {
    uint8_t pin;
    const char* name;
    std::optional<HexaServo> device;
};
}  // namespace

class ServoCalibration : public NRLOpMode {
public:
    void init() override { publishTelemetry(millis(), true); }

    void loop() override {
        const uint32_t now = millis();
        if (gamepad1.pressed(BTN_Y) || gamepad1.pressed(BTN_LT)) {
            detachAndLatch("DETACHED");
        } else if (NRLComms::isInputStale()) {
            detachAndLatch("LINK LOST");
        }
        if (_latched) {
            publishTelemetry(now);
            return;
        }

        const bool arm = gamepad1.pressed(BTN_X);
        const bool grip = gamepad1.pressed(BTN_A);
        const bool up = gamepad1.pressed(BTN_DPAD_UP);
        const bool down = gamepad1.pressed(BTN_DPAD_DOWN);
        const bool lb = gamepad1.pressed(BTN_LB);
        const bool rb = gamepad1.pressed(BTN_RB);
        const bool idle = !(arm || grip || up || down || lb || rb);

        if (_releaseRequired) {
            if (idle) _releaseRequired = false;
            publishTelemetry(now);
            return;
        }

        if (!_enabled && !lb && !rb && !up && !down && arm != grip) {
            _selected = arm ? 0 : 1;
        }

        if (lb || rb) {
            if (!_enabled && lb && rb && !arm && !grip && !up && !down) {
                if (!_holdingEnable) {
                    _holdingEnable = true;
                    _enableSince = now;
                } else if (static_cast<uint32_t>(now - _enableSince) >= kEnableHoldMs) {
                    enableSelected();
                    _holdingEnable = false;
                    _releaseRequired = true;
                }
            } else {
                _holdingEnable = false;
            }
        } else if (_enabled && up != down) {
            // Move continuously while held; the platform loop runs at 100 Hz.
            const float dir = up ? 1.0f : -1.0f;
            const float minAngle = _selected == 0 ? kArmMinAngle : kGripperMinAngle;
            const float maxAngle = _selected == 0 ? kArmMaxAngle : kGripperMaxAngle;
            const float next = _selectedDevice().getPosition() + dir * kRateDegPerSec * 0.01f;
            _selectedDevice().setPosition(next < minAngle ? minAngle :
                                          (next > maxAngle ? maxAngle : next));
        } else {
            _holdingEnable = false;
        }

        publishTelemetry(now);
    }

    void stop() override { detachAndLatch("STOPPED"); }

private:
    CalibrationServo _arm{SERVO_1, "ARM S1 / J18"};
    CalibrationServo _gripper{SERVO_3, "GRIPPER S3 / J17"};
    uint8_t _selected = 0;
    bool _enabled = false;
    bool _latched = false;
    bool _releaseRequired = true;
    bool _holdingEnable = false;
    uint32_t _enableSince = 0;
    uint32_t _lastTelemetry = 0;
    const char* _stopReason = "STOPPED";

    HexaServo& _selectedDevice() { return *(_selected == 0 ? _arm.device : _gripper.device); }

    void enableSelected() {
        CalibrationServo& selected = _selected == 0 ? _arm : _gripper;
        HexaServoConfig config{};
        config.signalPin = selected.pin;
        config.minAngle = _selected == 0 ? kArmMinAngle : kGripperMinAngle;
        config.maxAngle = _selected == 0 ? kArmMaxAngle : kGripperMaxAngle;
        config.startAngle = kTrialAngle;
        config.settleMs = 0;
        selected.device.emplace(config);
        selected.device->setRampRate(kRateDegPerSec);
        if (!selected.device->begin()) {
            detachAndLatch("INIT FAILED");
            return;
        }
        _enabled = true;
    }

    void detachAndLatch(const char* reason) {
        if (_latched) return;
        if (_arm.device) _arm.device->detach();
        if (_gripper.device) _gripper.device->detach();
        _enabled = false;
        _holdingEnable = false;
        _latched = true;
        _stopReason = reason;
    }

    void publishTelemetry(uint32_t now, bool force = false) {
        if (!force && static_cast<uint32_t>(now - _lastTelemetry) < kTelemetryMs) return;
        _lastTelemetry = now;
        const CalibrationServo& selected = _selected == 0 ? _arm : _gripper;
        telemetry.addData("selected", selected.name);
        telemetry.addData("state", _latched ? _stopReason :
                          (_enabled ? "ENABLED - HOLD UP/DOWN" :
                           (_holdingEnable ? "HOLD LB+RB" : "TRIAL ONLY")));
        telemetry.addData("angle", selected.device && selected.device->isReady() ?
                          selected.device->getPosition() : kTrialAngle);
        telemetry.addData("limits", _selected == 0 ? "arm 20-160 deg" : "grip 60-120 deg");
        telemetry.addData("rate", "30 deg/sec while held");
        telemetry.addData("hint", _latched ? "RESTART OPMODE" :
                          (_enabled ? "UP/DOWN; Y:OFF" : "X:ARM A:GRIP; LB+RB 1s:ON"));
    }
};

REGISTER_OPMODE(ServoCalibration, "ServoCalibration", TELEOP);

#include "NRL.h"
#include <optional>

// Calibrate ONE servo per run. No drive motors are created or commanded.
// Confirmed wiring: arm J19/S2 = SERVO_2; gripper J17/S3 = SERVO_3.
//
// INIT/START: no servo begin(), attach(), or position commands.
// X / A: select arm / gripper before enabling.
// D-pad UP / DOWN: change the selected trial/command by 1 degree per tap.
// Hold LB + RB alone for 1 second: enable the selected servo at its trial angle.
// Y: detach both immediately on the next loop tick and latch until restart.
// LT (controller exit), stale input, and stop() also detach and latch.
// Restart the OpMode to calibrate the other servo or to enable again after STOP.
//
// IMPORTANT: 90 degrees is an UNVERIFIED trial value, not a safe pose or a
// measured position. begin() commands the selected angle immediately; the first
// movement may be large. Support/unload the mechanism before enabling it.
// 0..180 is the driver's nominal range, NOT the mechanism's calibrated limits.
// Detaching removes holding torque, so an unsupported lifting arm can drop.
// Telemetry reports last commanded angles, not physical position feedback.

namespace {
constexpr float kTrialAngle = 90.0f;  // No output until deliberate enable.
constexpr float kStepDeg = 1.0f;
constexpr uint32_t kEnableHoldMs = 1000;
constexpr uint32_t kTelemetryMs = 200;  // 5 Hz; SDK handles transmission.

struct CalibrationServo {
    uint8_t pin;
    const char* name;
    float trialAngle = kTrialAngle;
    // In-place storage: configure startAngle before begin(), without heap
    // allocation or a brief pulse at a different angle. Construct only once.
    std::optional<HexaServo> device;
};
}  // namespace

class ServoCalibration : public NRLOpMode {
public:
    void init() override {
        // Deliberately defer begin(): it would move the servos during INIT.
        publishTelemetry(millis(), true);
    }

    void loop() override {
        const uint32_t now = millis();

        // Detach takes priority over every selection, enable, and motion input.
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

        // Require release after startup, selection, or enable. A held button
        // cannot accidentally carry an action into the next state.
        if (_releaseRequired) {
            if (idle) _releaseRequired = false;
            publishTelemetry(now);
            return;
        }

        CalibrationServo& selected = _selected == 0 ? _arm : _gripper;
        if (lb || rb) {
            // Enabling is allowed only with both bumpers and no other controls.
            // Changing selection/angle cancels the hold rather than moving.
            if (!_enabled && lb && rb && !arm && !grip && !up && !down) {
                if (!_holdingEnable) {
                    _holdingEnable = true;
                    _enableSince = now;
                } else if (static_cast<uint32_t>(now - _enableSince) >= kEnableHoldMs) {
                    enableSelected(selected);
                    _holdingEnable = false;
                    _releaseRequired = true;
                }
            } else {
                _holdingEnable = false;
            }
            publishTelemetry(now);
            return;
        }
        _holdingEnable = false;

        // Lock selection once enabled: the other servo remains uninitialized.
        if (arm || grip) {
            if (!_enabled && arm != grip && !up && !down) {
                if (arm && gamepad1.justPressed(BTN_X)) _selected = 0;
                if (grip && gamepad1.justPressed(BTN_A)) _selected = 1;
                _releaseRequired = true;
            }
        } else if (up != down) {
            // Edge-triggered only: holding the D-pad does not repeat movement.
            if (up && gamepad1.justPressed(BTN_DPAD_UP)) step(selected, kStepDeg);
            if (down && gamepad1.justPressed(BTN_DPAD_DOWN)) step(selected, -kStepDeg);
        }

        publishTelemetry(now);
    }

    void stop() override {
        detachAndLatch("STOPPED");
    }

private:
    CalibrationServo _arm{SERVO_2, "ARM S2"};
    CalibrationServo _gripper{SERVO_3, "GRIPPER S3"};
    uint8_t _selected = 0;
    bool _enabled = false;
    bool _latched = false;
    bool _releaseRequired = true;
    bool _holdingEnable = false;
    uint32_t _enableSince = 0;
    uint32_t _lastTelemetry = 0;
    const char* _stopReason = "STOPPED";

    void enableSelected(CalibrationServo& selected) {
        HexaServoConfig config{};
        config.signalPin = selected.pin;
        config.startAngle = selected.trialAngle;
        config.settleMs = 0;  // Hold until explicitly detached; no auto-release.
        selected.device.emplace(config);
        if (!selected.device->begin()) {
            detachAndLatch("INIT FAILED");
            return;
        }
        _enabled = true;
    }

    void step(CalibrationServo& selected, float delta) {
        float next = selected.trialAngle + delta;
        if (next < 0.0f) next = 0.0f;
        if (next > 180.0f) next = 180.0f;
        if (next == selected.trialAngle) return;
        selected.trialAngle = next;
        if (_enabled) selected.device->setPosition(next);
    }

    void detachAndLatch(const char* reason) {
        if (_latched) return;
        // detach() is also safe to call after a failed begin(). Never call
        // setPosition()/attach() after this: either could restore PWM.
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
                          (_enabled ? "ENABLED" : (_holdingEnable ? "HOLD LB+RB" : "TRIAL ONLY")));
        telemetry.addData("trial deg", selected.trialAngle);
        if (_arm.device && _arm.device->isReady()) {
            telemetry.addData("arm cmd", _arm.device->getPosition());
        } else {
            telemetry.addData("arm cmd", "unknown");
        }
        if (_gripper.device && _gripper.device->isReady()) {
            telemetry.addData("grip cmd", _gripper.device->getPosition());
        } else {
            telemetry.addData("grip cmd", "unknown");
        }
        telemetry.addData("step deg", kStepDeg);
        telemetry.addData("hint", _latched ? "RESTART OPMODE" :
                          (_enabled ? "UP/DN; Y:OFF" : "LB+RB 1s:ON"));
        telemetry.addData("select", _enabled || _latched ? "LOCKED" : "X:arm A:grip");
    }
};

REGISTER_OPMODE(ServoCalibration, "ServoCalibration", TELEOP);

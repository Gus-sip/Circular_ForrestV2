#pragma once

// Stub power manager for the CHIP FOREST bench harness. No gating MOSFETs exist on
// the bench yet - it's one always-on 3V3 rail - but the API shape here is what the
// real power-gated PowerManager will expose, so rail dependencies are expressed in
// code now instead of getting untangled later when the FETs show up.
class PowerManager {
public:
  void enable3V3Sensors();
  void disable3V3Sensors();
  bool sensors3V3Enabled() const { return _sensors3V3; }

private:
  bool _sensors3V3 = false;
};

#pragma once

// ╔═══════════════════════════════════════════════════════════════╗
// ║  chofu_wp - ESPHome external component                        ║
// ║  Atlantic Aurea 5 Hybrid / Chofu 0x19 protocol @ 666 baud     ║
// ╚═══════════════════════════════════════════════════════════════╝
//
// Protocol layer ported 1:1 from the WORKING ATmega2560 sketch from the Tweakers topic
// (originally by WackoH, v1.0 02-03-2025 and 0.1Beta 19-03-2025; JGC's adaptation
// of it was our first basis). That is reverse-engineered code, proven on real hardware
// code - hence the frame layout, timing and CRC were copied exactly. See ../../README.md and the memory 'aurea-serieel-protocol'.
//
// The CONTROL: "give me X°C supply" -> this controller picks the stage itself
// (0 to max_stand; with step modulation so that the compressor does not wake the
// neighbours at full throttle). Where that X comes from you choose with the mode:
//   auto (fixed setpoint) - fixed setpoint ("Supply setpoint")
//   heating curve - from the outdoor temperature of the heat pump
//   room temperature - heating curve, corrected with a room temperature (from
//                 an OpenTherm thermostat, Home Assistant or a script)
//   manual      - no control, a fixed stage

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <cmath>

namespace esphome {
namespace chofu_wp {

class ChofuWP : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;    // RX + TX rotation (continuous, master/slave timing)
  void update() override;  // Control, every 5 s
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ── Status (read out by template sensors in the YAML) ──
  float get_supply() const { return t_supply_; }
  float get_return() const { return t_return_; }
  float get_outside() const { return t_outside_; }
  float get_delta_t() const { return t_supply_ - t_return_; }
  int get_stand() const { return stand_; }             // What WE command
  int get_max_stand() const { return max_stand_; }
  float get_setpoint() const { return setpoint_; }      // The fixed setpoint (auto mode)
  int get_wp_speed() const { return wp_speed_; }        // What the heat pump reports (ID3 byte 9, actually the current)
  // Extra values from the outdoor unit, per WackoH's byte map (Tweakers and GitHub issue #1). NAN or -1 until the message has arrived.
  float get_freq() const { return freq_; }               // compressor frequency in Hz (ID3 byte 3 x 2/3)
  float get_current() const { return current_; }         // current in A (ID3 byte 9 / 5)
  float get_fan_rpm() const { return fan_rpm_; }         // actual fan speed (ID3 byte 4 x 10)
  float get_fan_set_rpm() const { return fan_set_rpm_; } // fan setpoint (ID1 byte 10 x 10)
  float get_eev() const { return eev_; }                 // expansion valve (ID3 byte 6 x 2 pulses)
  int get_pump_byte() const { return pump_byte_; }       // ID3 byte 13 (24 off, 152 on at WackoH; constant 24 on our units)
  int get_defrost_flag() const { return defrost_flag_; } // ID1 byte 4: 1 = defrosting
  int get_run_status() const { return run_status_; }     // ID1 byte 3: 0x50 heating, 0x40 stop/defrost, 0 off
  int get_status8() const { return status8_; }           // ID3 byte 8: bit 0 = defrosting, bit 6 = 60 min after the first start
  float get_t_defrost() const { return t_defrost_; }     // defrost/flush temperature (ID2 byte 11, raw/2 - 41)
  float get_t_suction() const { return t_suction_; }     // suction temperature (ID2 byte 12, raw/2 - 41)
  float get_t_discharge() const { return t_discharge_; } // discharge gas temperature (ID2 byte 13, directly in degrees)
  float get_t_water() const { return t_water_; }         // water-side temperature (ID2 byte 14, raw/2 - 41)
  float get_power() const { return wp_power_w_; }       // Real power (W)
  bool get_defrost() const { return defrost_; }
  bool get_running() const { return stand_ > 0; }
  bool get_comm_ok() const {
    return last_valid_rx_ms_ != 0 && (millis() - last_valid_rx_ms_) < COMM_TIMEOUT_MS;
  }

  // ── Settings (set from template numbers/switch/select) ──
  void set_setpoint(float v);
  void set_system_on(bool on);
  // Mode: see the top of this file.
  enum Mode : uint8_t { MODE_AUTO, MODE_CURVE, MODE_ROOM, MODE_MANUAL };
  void set_mode(Mode m);
  Mode get_mode() const { return mode_; }
  void set_auto_mode(bool a) { set_mode(a ? MODE_AUTO : MODE_MANUAL); }
  void set_manual_stand(int s);
  void set_cooling(bool c);                       // Cooling mode (mode 0x02)
  bool get_cooling() const { return cooling_; }
  void set_raw_debug(bool b) { raw_debug_ = b; }  // FRAME hex log on/off (web toggle)
  void force_step() { last_step_ms_ = 0; }  // Next cycle may switch immediately

  // ── Control refinements (all settable from HA; 0 = off) ──
  // Averaging: the control looks at the average of supply and return over
  // this many seconds. With the compressor running the supply saws about every
  // minute 3-4 °C; a single measurement at the moment of a step is then chance. Max 200 s.
  void set_avg_window(float seconds) { avg_s_ = seconds; }
  // Lookahead: judge the supply as it is expected to be in this many minutes
  // (supply + rise per minute x this time). If it gets there by itself, then
  // no extra stage; if it overshoots by a wide margin, step back a stage now.
  // Only from stage 2: stopping (1 -> 0) stays tied to the real supply.
  void set_lookahead(float minutes) { lookahead_min_ = minutes; }
  // Do not step up while the supply has risen more than this many
  // degrees in the last 5 minutes: the previous step is then still taking effect.
  void set_rise_limit(float c) { rise_limit_ = c; }
  // Stepping down above stage 1 does not have to wait the full step interval:
  // down is the safe side. 0 = same interval as up.
  void set_down_interval(uint32_t ms) { down_interval_ms_ = ms; }
  // Stage 1 -> 0 with heat demand: first want it this long in a row, and no prediction that it has to come back.
  void set_stop_delay(uint32_t ms) { stop_delay_ms_ = ms; }
  // The target moves towards MORE demand by at most this many °C per minute; less
  // demand applies immediately. A jump in the target thus does not push the unit straight to
  // a high stage that it overshoots.
  void set_setpoint_slew(float c_per_min) { slew_c_per_min_ = c_per_min; }
  // NEW: dead band (°C) around the supply target in which the stage is held. The supply
  // ripples by several °C with the compressor running, so 0.3 made the stage hunt.
  void set_band(float c) { band_ = c; }

  // ── Heating curve (heating curve and room temperature mode) ──
  // Two points: the supply at -10 °C and at +15 °C outdoors, straight between and
  // constant outside. Above the heating limit there is no heat demand.
  void set_curve_cold(float v) { curve_cold_ = v; }
  void set_curve_warm(float v) { curve_warm_ = v; }
  // NEW: parallel shift of the curve with the room target (°C of supply per °C target, reference target)
  void set_curve_shift(float v) { curve_shift_ = v; }
  void set_curve_ref(float v) { curve_ref_ = v; }
  float get_room_i() const { return room_i_; }  // NEW: trim, for tuning
  void set_heating_limit(float c) { heat_limit_ = c; }
  float get_curve_supply() const;  // Heating curve at the current outdoor temperature

  // ── Room temperature (room temperature mode) ──
  // The room temperature comes from outside (the YAML sets it). Every time it is
  // set counts as "fresh", also the same value. If nothing arrives for longer than the
  // timeout, the unit switches off: without a room, no control.
  void set_room_temp(float v);
  void set_room_target(float v) { room_target_ = v; }
  void set_room_timeout(uint32_t ms) { room_timeout_ms_ = ms; }
  // Room influence: °C of supply per °C that the room is too cold (or too warm).
  void set_room_gain(float k) { room_gain_ = k; }
  // Room trim: °C of supply added per minute per °C of error, so that
  // the target is really reached. 0 = off (room influence only, like Anna).
  void set_room_integral(float ki) { room_ki_ = ki; }
  // Off as soon as the room is ROOM_HYST above the target; back on when it
  // drops ROOM_HYST below.
  void set_room_off_when_warm(bool b) { room_off_warm_ = b; }
  bool get_room_fresh() const;
  float get_room_temp() const { return room_temp_; }

  // The target the control is now actually regulating to (after heating curve, room and
  // ramping). NaN = no demand / not in automatic operation.
  float get_effective_setpoint() const { return sp_eff_; }
  // Why the control does what it does, for HA.
  const char *get_reason() const { return reason_; }

  // ── Compile-time config (from __init__.py) ──
  void set_setpoint_min(float v) { setpoint_min_ = v; }
  void set_setpoint_max(float v) { setpoint_max_ = v; }
  void set_cooling_min_supply(float v) { cooling_min_supply_ = v; }
  void set_dt_low(float v) { dt_low_ = v; }
  void set_dt_high(float v) { dt_high_ = v; }
  void set_max_stand(int v) { max_stand_ = v; }
  void set_step_interval(uint32_t ms) { step_interval_ms_ = ms; }

 protected:
  // ── Protocol ──
  void handle_rx_();
  void finish_frame_();  // Frame complete: check CRC + extract values
  void abort_frame_(const char *reason);  // Frame rejected in the header
  void maybe_send_();
  void build_data2_(uint8_t *out);
  static uint16_t crc_ccitt_(const uint8_t *data, uint8_t len);
  static bool valid_msg_length_(uint8_t len);

  // ── Control ──
  void run_control_();

  // Timing constants (identical to the sketches by WackoH/JGC)
  static const uint32_t SEND_DELAY_MS = 99;           // Transmit 99 ms after end of RX
  static const uint32_t SEND_TIMEOUT_MS = 2000;       // Fallback if RX fails to arrive
  static const uint32_t MIN_SEND_INTERVAL_MS = 300;   // Min. time between TXs
  static const uint32_t COMM_TIMEOUT_MS = 60000;      // No valid RX -> fail-safe
  // Hard collision protection: at 666 baud one byte takes ~15 ms, so 40 ms
  // of silence on the line guarantees that the heat pump is done transmitting.
  static const uint32_t LINE_IDLE_MS = 40;
  // Lookahead: only step back earlier if the expected supply ends up this many
  // degrees above the setpoint.
  static constexpr float BRAKE_OVERSHOOT_K = 1.0f;
  // Room temperature: hysteresis around the room target for "off when room is warm".
  static constexpr float ROOM_HYST = 0.2f;
  // Limits of the room trim (°C of supply).
  static constexpr float ROOM_I_MIN = -8.0f, ROOM_I_MAX = 12.0f;
  // NEW: the trim only integrates while the room error is below this (°C).
  static constexpr float ROOM_I_BAND = 1.0f;
  // Lower limit of a target from heating curve or room temperature: colder water
  // no longer heats anything.
  static constexpr float HEAT_MIN_SUPPLY = 20.0f;

  // ── Config / control parameters ──
  float setpoint_{35.0f};
  // Volledig technisch bereik uit de Chofu-datasheet (AEYC-0643XU),
  // "Operating Range / Leaving Water Temperature":
  //   heating ~60°C  |  cooling 6.5°C~
  // (Atlantic states 55°C, but that is their handover point to the gas boiler,
  //  not a hardware limit.)
  float setpoint_min_{6.5f}, setpoint_max_{60.0f};
  // Absolute frost protection when cooling - deliberately BELOW the factory limit
  // of 6.5°C, so it never gets in the way of legitimate operation.
  float cooling_min_supply_{5.0f};
  float dt_low_{4.0f}, dt_high_{8.0f};  // Healthy delta-T band (°C)
  // Adjustable up to 10; the default stays 7.
  // The disassembled Atlantic firmware (IC1, "Haddon M1.1") shows a
  // precomputed telegram table for stages 1 to 10 - all with valid
  // CRC-16 - and an internal stage variable that runs up to 40 with index =
  // stage/4. So stage 10 is really reachable in the original.
  // NOT proven for every outdoor unit: that table is in the INDOOR unit, and
  // Atlantic uses a shared code base across several models (both
  // MCUs demonstrably carry dead code from each other). JGC used 6 steps,
  // we 7, the table has 10. So raise it step by step and check whether
  // the sensors "Power" and "Speed" - which the heat pump itself reports -
  // actually rise along. If they stay flat, your unit does not pick up the higher
  // stages and the rest is table filler for a larger model.
  uint8_t max_stand_{7};
  uint32_t step_interval_ms_{120000};
  // Off until the switch in the YAML says otherwise. There is a gap between
  // the start-up of this component and the restoring of that switch; in
  // that gap the heat pump should be stopped.
  bool system_on_{false};
  Mode mode_{MODE_AUTO};
  bool cooling_{false};  // false = heating, true = cooling (experimental)
  uint8_t manual_stand_{1};

  // ── Control state ──
  uint8_t stand_{0};        // Commanded stage, 0 to max_stand_
  uint32_t last_step_ms_{0};
  // History of supply and return, a sample every 10 s: 40 slots = 400 s.
  static const uint8_t HIST_N = 40;
  float sup_hist_[HIST_N];
  float ret_hist_[HIST_N];
  uint8_t hist_idx_{0}, hist_cnt_{0};
  uint32_t hist_last_ms_{0};
  uint8_t avg_samples_() const;
  float avg_hist_(const float *h, uint8_t from, uint8_t n) const;
  float get_rise_() const;  // Rise over 5 min; NaN = too little history
  float get_rate_() const;  // Rise per minute of the average supply
  float avg_s_{0.0f};
  float lookahead_min_{0.0f};
  float rise_limit_{0.0f};
  bool rise_holding_{false};
  uint32_t down_interval_ms_{0};
  uint32_t stop_delay_ms_{0};
  uint32_t stop_wish_ms_{0};   // since when the control has uninterruptedly wanted 0 (0 = not)
  bool stop_hold_{false};      // only for the log line
  float slew_c_per_min_{0.0f};
  float band_{1.0f};  // NEW
  float sp_eff_{NAN};
  uint32_t sp_eff_ms_{0};
  const char *reason_{"starting up"};
  void set_reason_(const char *r);
  // Target from the mode, without ramping. NaN = no demand.
  float target_supply_(uint32_t now);

  // ── Heating curve / room temperature ──
  float curve_cold_{50.0f}, curve_warm_{30.0f};
  float curve_shift_{1.9f}, curve_ref_{20.0f};  // NEW
  float heat_limit_{18.0f};
  float room_temp_{NAN};
  uint32_t room_ms_{0};
  uint32_t room_timeout_ms_{30 * 60000};
  float room_target_{20.0f};
  float room_gain_{4.0f};
  float room_ki_{0.0f};
  float room_i_{0.0f};
  uint32_t room_i_ms_{0};
  bool room_off_warm_{false};
  // Hysteresis around the room setpoint for "off when the room is warm": off above setpoint + offset, on below setpoint - offset.
  float room_hyst_off_{0.2f};
  float room_hyst_on_{0.2f};
  uint32_t room_min_hold_ms_{0};
  uint32_t room_demand_ms_{0};
  bool room_demand_{true};
  bool above_heat_limit_{false};

  // ── Data from the heat pump ──
  float t_supply_{NAN}, t_return_{NAN}, t_outside_{NAN};
  uint8_t wp_speed_{0};
  float freq_{NAN}, current_{NAN}, fan_rpm_{NAN}, fan_set_rpm_{NAN}, eev_{NAN};
  float t_defrost_{NAN}, t_suction_{NAN}, t_discharge_{NAN}, t_water_{NAN};
  int pump_byte_{-1}, defrost_flag_{-1}, run_status_{-1}, status8_{-1};
  float wp_power_w_{0};
  bool defrost_{false};
  uint32_t last_valid_rx_ms_{0};

  // ── TX-rotatie ──
  uint8_t telegram_count_{0};
  uint32_t prev_msg_sent_ms_{0};
  uint32_t prev_msg_in_ended_ms_{0};
  bool is_receiving_{false};

  // ── RX-state machine ──
  enum RxState { WAIT_START, READ_HEADER, READ_PAYLOAD };
  RxState rx_state_{WAIT_START};
  uint8_t rx_hdr_idx_{0};
  uint8_t rx_id_{0};
  uint8_t rx_datalen_{0};
  uint8_t rx_idx_{0};

  // Diagnostics: hex-log every complete frame + CRC. Off by default; can be
  // turned on via the "Debug frames" switch in the web GUI / Home Assistant.
  bool raw_debug_{false};
  uint8_t frame_[40];        // Complete frame from 0x91
  uint8_t frame_len_{0};
  uint32_t last_byte_ms_{0}; // Time of last received byte (line-idle monitoring)
};

}  // namespace chofu_wp
}  // namespace esphome

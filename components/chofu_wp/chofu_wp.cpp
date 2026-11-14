#include "chofu_wp.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstring>

namespace esphome {
namespace chofu_wp {

static const char *const TAG = "chofu_wp";

// ═══════════════════════════════════════════════════════════════
//  The 4 rotating transmit telegrams (control box -> heat pump)
//  data0/1/3 are fixed; data2 is built each time (on/off + stage + CRC).
//  Bytes copied exactly from the working sketches by WackoH (and JGC), see chofu_wp.h.
// ═══════════════════════════════════════════════════════════════
static const uint8_t DATA0[] = {0x19, 0x00, 0x08, 0x00, 0x00, 0x00, 0xD9, 0xB5};
static const uint8_t DATA1[] = {0x19, 0x01, 0x0C, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0xAA, 0x35};
static const uint8_t DATA3[] = {0x19, 0x03, 0x08, 0xB2, 0x02, 0x00, 0xC1, 0x9A};

// Valid message lengths from the heat pump (msgLength = length byte + 1), from WackoH's sketches.
static const uint8_t VALID_LENGTHS[] = {13, 14, 19, 21};

// ═══════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════

void ChofuWP::setup() {
  ESP_LOGI(TAG, "Chofu 0x19 protocol started (666 baud, 4 rotating telegrams)");
}

void ChofuWP::dump_config() {
  ESP_LOGCONFIG(TAG, "Chofu WP (666 baud):");
  ESP_LOGCONFIG(TAG, "  Supply setpoint: %.1f°C (range %.0f-%.0f)", setpoint_, setpoint_min_,
                setpoint_max_);
  ESP_LOGCONFIG(TAG, "  Operating mode: %s", cooling_ ? "cooling" : "heating");
  ESP_LOGCONFIG(TAG, "  Control: delta-T band %.1f-%.1f°C, max stage %u, step every %us", dt_low_,
                dt_high_, max_stand_, (unsigned) (step_interval_ms_ / 1000));
  ESP_LOGCONFIG(TAG, "  Heating curve: %.1f°C at -10, %.1f°C at +15, heating limit %.1f°C", curve_cold_,
                curve_warm_, heat_limit_);
  this->check_uart_settings(666);
}

// ═══════════════════════════════════════════════════════════════
//  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection)
//  CRC over the whole message incl. the 2 CRC bytes == 0 on error-free reception.
// ═══════════════════════════════════════════════════════════════
uint16_t ChofuWP::crc_ccitt_(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t) data[i] << 8;
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x8000) ? (uint16_t) ((crc << 1) ^ 0x1021) : (uint16_t) (crc << 1);
  }
  return crc;
}

bool ChofuWP::valid_msg_length_(uint8_t len) {
  for (uint8_t v : VALID_LENGTHS)
    if (len == v)
      return true;
  return false;
}

// ═══════════════════════════════════════════════════════════════
//  loop(): process RX bytes + (at the right moments) send a telegram
// ═══════════════════════════════════════════════════════════════
void ChofuWP::loop() {
  this->handle_rx_();
  this->maybe_send_();
}

// ═══════════════════════════════════════════════════════════════
//  RX - 0x91 messages from the heat pump, per ID (state machine from WackoH's sketches)
// ═══════════════════════════════════════════════════════════════
void ChofuWP::handle_rx_() {
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b))
      break;
    is_receiving_ = true;
    last_byte_ms_ = millis();  // For the line-idle monitoring in maybe_send_()

    switch (rx_state_) {
      case WAIT_START:
        if (b == 0x91) {
          rx_state_ = READ_HEADER;
          rx_hdr_idx_ = 0;
          frame_[0] = b;
          frame_len_ = 1;
        }
        break;

      case READ_HEADER:
        if (frame_len_ < sizeof(frame_))
          frame_[frame_len_++] = b;
        if (rx_hdr_idx_ == 0) {
          rx_id_ = b;
          if (rx_id_ > 3) {  // ID must be 0-3
            this->abort_frame_("invalid ID");
            break;
          }
        } else if (rx_hdr_idx_ == 1) {
          uint8_t msg_length = b + 1;
          if (!valid_msg_length_(msg_length)) {
            this->abort_frame_("invalid length");
            break;
          }
          rx_datalen_ = msg_length - 3;
          rx_idx_ = 0;
          rx_state_ = READ_PAYLOAD;
        }
        rx_hdr_idx_++;
        break;

      case READ_PAYLOAD:
        if (frame_len_ < sizeof(frame_))
          frame_[frame_len_++] = b;
        rx_idx_++;
        if (rx_idx_ >= rx_datalen_) {
          // Frame complete: 0x91 + ID + len + <datalen> payload (last 2 = CRC).
          // NO separate end byte - the next byte is already the next telegram.
          this->finish_frame_();
          rx_state_ = WAIT_START;
        }
        break;
    }
  }
}

// Frame complete: validate CRC (CRC over the whole frame == 0) + extract values.
void ChofuWP::finish_frame_() {
  is_receiving_ = false;
  prev_msg_in_ended_ms_ = millis();  // ALWAYS - this way the TX timing follows the heat pump's cadence

  const uint16_t crc = crc_ccitt_(frame_, frame_len_);

  if (raw_debug_) {
    char hex[sizeof(frame_) * 3 + 1];
    for (size_t i = 0; i < frame_len_; i++)
      sprintf(hex + i * 3, "%02X ", frame_[i]);
    hex[frame_len_ * 3] = '\0';
    ESP_LOGD(TAG, "FRAME ID%u [%u] %s crc=%04X %s", rx_id_, (unsigned) frame_len_, hex, crc,
             crc == 0 ? "OK" : "FAIL");
  }

  if (crc != 0)
    return;  // Ignore corrupt frame

  last_valid_rx_ms_ = millis();

  // Temperatures: signed 16-bit LITTLE-endian (LSB first), /10.
  switch (rx_id_) {
    case 1:
      // ID1 idx4 was marked as "defrost", but measurements show that
      // this byte simply becomes 0x01 as soon as the unit runs (and 0x00 when it is
      // stopped) - it is a run status, not defrost. Until we know the real
      // defrost byte, we no longer publish it as defrost.
	    // To the byte map of WackoH (Tweakers, GitHub issue #1), byte 4 is indeed "defrosting active" (0/1) and byte 3 the
      // operating status (0x50 heating, 0x40 stop before or during defrosting, 0x00 off); byte 10 is the fan setpoint (x10 rpm).
      run_status_ = frame_[3];
      defrost_flag_ = frame_[4];
      fan_set_rpm_ = frame_[10] * 10.0f;
      break;
    case 2:
      t_supply_ = (int16_t) ((frame_[4] << 8) | frame_[3]) / 10.0f;
      t_return_ = (int16_t) ((frame_[6] << 8) | frame_[5]) / 10.0f;
      t_outside_ = (int16_t) ((frame_[8] << 8) | frame_[7]) / 10.0f;
	  if (frame_[9] == 0) {  // gegevenspagina (01 = constante pagina), naar WackoH: T = raw/2 - 41
        t_defrost_ = frame_[11] / 2.0f - 41.0f;
        t_suction_ = frame_[12] / 2.0f - 41.0f;
        t_discharge_ = (float) frame_[13];
        t_water_ = frame_[14] / 2.0f - 41.0f;
      }
      break;
    case 3:
      wp_speed_ = frame_[9];
      // 2-11-2026 Verified against a Shelly (48-hour period): byte 10 tracks the power, with each increment representing ~24 W
      // (+ ~100 W for the pump/electronics); byte 10 represents the current in 0.1 A units (per WackoH). Byte 9 follows a similar trend
      // (approximately 1.16 × byte 10), but it does NOT represent current in 0.2 A units, as previously stated here.
      wp_power_w_ = 25.6f * frame_[10];
	    // According to WackoH’s byte map: byte 3 = compressor frequency (x 2/3 Hz), byte 4 = fan (x 10 rpm),
      // byte 6 = expansion valve (x2 pulses), byte 8 = status bits (bit 0 defrost), byte 9 = current (/5 A).
      freq_ = frame_[3] * 2.0f / 3.0f;
      fan_rpm_ = frame_[4] * 10.0f;
      eev_ = frame_[6] * 2.0f;
      status8_ = frame_[8];
      current_ = frame_[10] / 10.0f;
      pump_byte_ = frame_[13];
      break;
    default:
      break;
  }
}

// Frame rejected in the header (invalid ID/length). Important: update the
// timing here too, otherwise is_receiving_ gets stuck and maybe_send_()
// falls back on the blind timeout - exactly what caused collisions.
void ChofuWP::abort_frame_(const char *reason) {
  if (raw_debug_)
    ESP_LOGD(TAG, "RX aborted (%s) - resync", reason);
  is_receiving_ = false;
  prev_msg_in_ended_ms_ = millis();
  rx_state_ = WAIT_START;
}

// ═══════════════════════════════════════════════════════════════
//  TX - send the next telegram when the master/slave timing allows it
// ═══════════════════════════════════════════════════════════════
void ChofuWP::maybe_send_() {
  const uint32_t now = millis();

  // HARD collision protection: never transmit while bytes are still trickling in.
  // Does not rely on the parser state (which can derail on a corrupt frame),
  // but purely on the line itself. Prevents our telegram from landing in the
  // middle of a heat pump frame.
  if (now - last_byte_ms_ < LINE_IDLE_MS)
    return;

  const bool after_rx = (now - prev_msg_in_ended_ms_ >= SEND_DELAY_MS) && !is_receiving_;
  const bool timed_out = (now - prev_msg_in_ended_ms_ >= SEND_TIMEOUT_MS);
  const bool spaced = (now - prev_msg_sent_ms_ >= MIN_SEND_INTERVAL_MS);
  if (!((after_rx || timed_out) && spaced))
    return;

  switch (telegram_count_) {
    case 0:
      this->write_array(DATA0, sizeof(DATA0));
      break;
    case 1:
      this->write_array(DATA1, sizeof(DATA1));
      break;
    case 2: {
      uint8_t data2[8];
      this->build_data2_(data2);
      this->write_array(data2, sizeof(data2));
      break;
    }
    case 3:
      this->write_array(DATA3, sizeof(DATA3));
      break;
  }

  telegram_count_ = (telegram_count_ + 1) % 4;
  prev_msg_sent_ms_ = now;
}

// data2: 19 02 08 <mode> <stage> 00 <crc_hi> <crc_lo>
// mode: 0x00 = off, 0x01 = heating, 0x02 = cooling.
// NOTE: 0x02 comes from a single forum post and is not used in the sketches - so it is
// experimental. The heat pump may simply ignore it.
void ChofuWP::build_data2_(uint8_t *out) {
  out[0] = 0x19;
  out[1] = 0x02;
  out[2] = 0x08;
  if (stand_ == 0) {
    out[3] = 0x00;  // off
    out[4] = 0x00;
  } else {
    out[3] = cooling_ ? 0x02 : 0x01;  // cooling or heating
    out[4] = stand_;
  }
  out[5] = 0x00;
  uint16_t crc = crc_ccitt_(out, 6);
  out[6] = (crc >> 8) & 0xFF;  // high byte
  out[7] = crc & 0xFF;         // low byte
}

// ═══════════════════════════════════════════════════════════════
//  Control - every 5 s (update_interval).
//  First the target (from the mode), then supply -> stage.
// ═══════════════════════════════════════════════════════════════
void ChofuWP::update() { this->run_control_(); }

void ChofuWP::set_reason_(const char *r) {
  if (strcmp(r, reason_) == 0)
    return;
  reason_ = r;
  ESP_LOGI(TAG, "Control: %s", r);
}

uint8_t ChofuWP::avg_samples_() const {
  // One sample per 10 s. Window 0 = no averaging = the latest sample.
  if (avg_s_ <= 0.0f)
    return 1;
  int n = (int) lroundf(avg_s_ / 10.0f);
  return (uint8_t) clamp(n, 1, 20);
}

// Average of n samples, starting "from" samples back (0 = newest).
// Uses fewer samples if the history is still shorter; NaN samples do not count.
float ChofuWP::avg_hist_(const float *h, uint8_t from, uint8_t n) const {
  if (hist_cnt_ <= from)
    return NAN;
  if ((int) from + n > hist_cnt_)
    n = (uint8_t) (hist_cnt_ - from);
  float sum = 0.0f;
  uint8_t cnt = 0;
  for (uint8_t k = 0; k < n; k++) {
    const uint8_t idx = (uint8_t) ((hist_idx_ + HIST_N - 1 - from - k) % HIST_N);
    if (!std::isnan(h[idx])) {
      sum += h[idx];
      cnt++;
    }
  }
  return cnt ? sum / cnt : NAN;
}

float ChofuWP::get_rise_() const {
  // Average of 5 samples now versus 5 samples ~280 s earlier, scaled
  // to 300 s. A single sample would measure the sawing of the supply.
  if (hist_cnt_ < 33)
    return NAN;
  const float nieuw = avg_hist_(sup_hist_, 0, 5);
  const float oud = avg_hist_(sup_hist_, 28, 5);
  if (std::isnan(nieuw) || std::isnan(oud))
    return NAN;
  return (nieuw - oud) * 30.0f / 28.0f;
}

float ChofuWP::get_rate_() const {
  // Average over the window now versus the same window 60 s earlier.
  const uint8_t w = avg_samples_();
  if (hist_cnt_ < 6 + w)
    return NAN;
  const float nieuw = avg_hist_(sup_hist_, 0, w);
  const float oud = avg_hist_(sup_hist_, 6, w);
  if (std::isnan(nieuw) || std::isnan(oud))
    return NAN;
  return nieuw - oud;
}

float ChofuWP::get_curve_supply() const {
  if (std::isnan(t_outside_))
    return NAN;
  // Straight line between (-10, curve_cold_) and (+15, curve_warm_), constant outside.
  const float b = clamp(t_outside_, -10.0f, 15.0f);
  return curve_cold_ + (curve_warm_ - curve_cold_) * (b + 10.0f) / 25.0f;
}

bool ChofuWP::get_room_fresh() const {
  return room_ms_ != 0 && !std::isnan(room_temp_) && (millis() - room_ms_) < room_timeout_ms_;
}

float ChofuWP::target_supply_(uint32_t now) {
  if (mode_ == MODE_AUTO || cooling_) {
    // Cooling has no heating curve: then always the fixed setpoint.
    set_reason_(mode_ == MODE_AUTO ? "fixed setpoint" : "cooling: fixed setpoint");
    return setpoint_;
  }

  if (std::isnan(t_outside_)) {
    set_reason_("no outdoor temperature - unit off");
    return NAN;
  }
  // Heating limit with 1 °C hysteresis: the outdoor sensor is on the unit itself and
  // moves along with sun and defrosting.
  if (t_outside_ >= heat_limit_)
    above_heat_limit_ = true;
  else if (t_outside_ < heat_limit_ - 1.0f)
    above_heat_limit_ = false;
  if (above_heat_limit_) {
    set_reason_("above the heating limit - no heat demand");
    return NAN;
  }

  const float curve = get_curve_supply();
  if (mode_ == MODE_CURVE) {
    set_reason_("heating curve");
    return curve;
  }

  // ── Room temperature ──
  if (!get_room_fresh()) {
    room_i_ = 0.0f;
    room_i_ms_ = 0;
    set_reason_("room temperature missing or expired - unit off");
    return NAN;
  }
  const float fout = room_target_ - room_temp_;  // > 0 = too cold

  if (room_off_warm_) {
    // Minimum switching time: once a change in demand occurs, it remains in effect for at least this duration,
    // preventing appliances—such as those used for cooking—from causing frequent on/off cycling. 0 = previous behavior.
    const bool vast_houden = room_min_hold_ms_ > 0 && room_demand_ms_ != 0 && (now - room_demand_ms_) < room_min_hold_ms_;
    if (!vast_houden) {
      if (room_demand_ && fout < -room_hyst_off_) {
        room_demand_ = false;
        room_demand_ms_ = now;
      } else if (!room_demand_ && fout > room_hyst_on_) {
        room_demand_ = true;
        room_demand_ms_ = now;
      }
    }
  } else {
    room_demand_ = true;
    room_demand_ms_ = 0;
  }

  // Trim: once per minute, and only while there is demand.
  if (room_ki_ <= 0.0f) {
    room_i_ = 0.0f;
  } else if (room_i_ms_ == 0) {
    room_i_ms_ = now;
  } else if (now - room_i_ms_ >= 60000) {
    const float minuten = (now - room_i_ms_) / 60000.0f;
    room_i_ms_ = now;
    const float zonder = curve + room_gain_ * fout + room_i_;
    // Do not keep increasing if the target is already at a limit (anti-windup).
    const bool vast = (fout > 0 && zonder >= setpoint_max_) || (fout < 0 && zonder <= HEAT_MIN_SUPPLY);
    if (room_demand_ && !vast)
      room_i_ = clamp(room_i_ + room_ki_ * fout * minuten, ROOM_I_MIN, ROOM_I_MAX);
  }

  if (!room_demand_) {
    set_reason_("room warm enough - unit off");
    return NAN;
  }
  set_reason_("room temperature");
  return clamp(curve + room_gain_ * fout + room_i_, HEAT_MIN_SUPPLY, setpoint_max_);
}

void ChofuWP::run_control_() {
  const uint32_t now = millis();

  // Keep a history of supply and return, also when stopped.
  if (!std::isnan(t_supply_) && (hist_last_ms_ == 0 || now - hist_last_ms_ >= 10000)) {
    sup_hist_[hist_idx_] = t_supply_;
    ret_hist_[hist_idx_] = t_return_;
    hist_idx_ = (uint8_t) ((hist_idx_ + 1) % HIST_N);
    if (hist_cnt_ < HIST_N)
      hist_cnt_++;
    hist_last_ms_ = now;
  }

  // Master switch off -> everything off
  if (!system_on_) {
    stand_ = 0;
    sp_eff_ = NAN;
    set_reason_("system off");
    return;
  }

  // Manual mode -> stage directly
  if (mode_ == MODE_MANUAL) {
    stand_ = clamp<uint8_t>(manual_stand_, 0, max_stand_);
    sp_eff_ = NAN;
    set_reason_("manual");
    return;
  }

  // Fail-safe: no valid telegram -> do not keep blindly requesting power
  if (last_valid_rx_ms_ == 0 || (now - last_valid_rx_ms_) > COMM_TIMEOUT_MS) {
    if (stand_ != 0)
      ESP_LOGW(TAG, "No valid communication - back to stage 0");
    stand_ = 0;
    set_reason_("no communication with the heat pump");
    return;
  }

  if (std::isnan(t_supply_) || std::isnan(t_return_))
    return;  // No data yet; keep the current stage

  // Control temperatures: averaged over the averaging window; without a window or
  // without history the raw value.
  float rs = t_supply_;
  float rr = t_return_;
  if (avg_s_ > 0.0f) {
    const float rs_gem = avg_hist_(sup_hist_, 0, avg_samples_());
    const float rr_gem = avg_hist_(ret_hist_, 0, avg_samples_());
    if (!std::isnan(rs_gem)) rs = rs_gem;
    if (!std::isnan(rr_gem)) rr = rr_gem;
  }

  // ── The target ──
  // Only towards MORE demand does it move gradually; less demand applies immediately.
  // After standstill the ramp starts at the current supply, otherwise a
  // cold start immediately sees the whole difference as error and steps up to a
  // stage that it later overshoots.
  const float doel = this->target_supply_(now);
  if (std::isnan(doel)) {
    sp_eff_ = NAN;
  } else if (slew_c_per_min_ <= 0.0f) {
    sp_eff_ = doel;
  } else {
    if (std::isnan(sp_eff_))
      sp_eff_ = cooling_ ? fmaxf(doel, rs) : fminf(doel, rs);
    const float max_stap = slew_c_per_min_ * (float) (now - sp_eff_ms_) / 60000.0f;
    if (cooling_)
      sp_eff_ = doel < sp_eff_ ? fmaxf(doel, sp_eff_ - max_stap) : doel;
    else
      sp_eff_ = doel > sp_eff_ ? fminf(doel, sp_eff_ + max_stap) : doel;
    // Never start below the current supply (when cooling: above it). After a
    // restart the target was at an old stored setpoint (17-10-2026: 20 °C
    // while the supply was 22); ramping up from there kept the unit idle for minutes
    // without that protecting against anything.
    if (cooling_)
      sp_eff_ = fminf(sp_eff_, fmaxf(doel, rs));
    else
      sp_eff_ = fmaxf(sp_eff_, fminf(doel, rs));
  }
  sp_eff_ms_ = now;

  // Cooling reverses everything: then you want the supply to go DOWN to the
  // setpoint, and the return is warmer than the supply. By defining both quantities
  // mode-dependently, the control table below stays identical:
  // err > 0 = more power needed, dt = how much the heat pump actually delivers.
  // No demand (target NaN) counts as "far too warm": wind down calmly to 0.
  const float sp = sp_eff_;
  const float dt = cooling_ ? (rr - rs) : (rs - rr);
  const float err = std::isnan(sp) ? -100.0f : (cooling_ ? (rs - sp) : (sp - rs));
  const float band = 0.3f;                       // dead band around setpoint

  // Absolute frost protection when cooling (default 5°C, below the factory limit
  // of 6.5°C - so it never gets in the way of legitimate operation). On the raw
  // supply itself: an average would be too late.
  if (cooling_ && t_supply_ < cooling_min_supply_) {
    if (stand_ != 0)
      ESP_LOGW(TAG, "Cooling: supply %.1f°C too low - stop", t_supply_);
    stand_ = 0;
    return;
  }

  // ── Choose direction: +1 more / -1 less / 0 hold ──
  // Delta-T tempers the choice: supply too cold BUT delta-T already high -> HOLD.
  // (The return warms up by itself; extra throttle would overshoot.) Deliberately the reverse
  // of the old .ino formula, which actually added power at high delta-T.
  int8_t dir = 0;
  if (err > band) {                       // supply too low -> want more...
    dir = (dt >= dt_high_) ? 0 : +1;      // ...unless delta-T is already too high
  } else if (err < -band) {               // supply too high -> less
    dir = -1;
  } else {                                // around setpoint
    if (dt > dt_high_)                     // delivering too aggressively -> throttle back
      dir = -1;
  }

  // ── Lookahead ──
  // The compressor takes effect slowly: a stage later the supply is only really
  // different, and at stage 3-4 it rises 1-2 °C per minute. Waiting until it is
  // above the setpoint is then too late.
  bool vroeg_remmen = false;
  if (lookahead_min_ > 0.0f && !std::isnan(sp)) {
    const float rate = get_rate_();
    if (!std::isnan(rate)) {
      const float rs_verwacht = rs + rate * lookahead_min_;
      const float err_verwacht = cooling_ ? (rs_verwacht - sp) : (sp - rs_verwacht);
      if (dir > 0 && stand_ > 0 && err_verwacht <= band) {
        dir = 0;                                      // gets there by itself: no extra stage
      } else if (dir <= 0 && stand_ > 1 && err_verwacht < -BRAKE_OVERSHOOT_K) {
        dir = -1;                                     // overshoots by a wide margin: step back a stage now
        vroeg_remmen = true;
      }
    }
  }

  // ── Stopping (stage 1 -> 0) with heat demand: first want it for some time in a row, and only if the
  // supply would not immediately be too low again after a stop ──
  // After a stop the supply drops to the return within ~1 minute. Predicted supply over the
  // lookahead time: return + (supply - return) * exp(-t/60 s). If that is below the setpoint minus the dead
  // band, it would restart immediately, so it does not stop. No heat demand (sp NaN) and cooling
  // keep the old behaviour; with "Stop delay" at 0 everything is as before.
  const bool wil_stop = !cooling_ && !std::isnan(sp) && stand_ == 1 && dir < 0;
  if (wil_stop && stop_delay_ms_ > 0) {
    if (stop_wish_ms_ == 0)
      stop_wish_ms_ = now;
    const float t_min = lookahead_min_ > 0.0f ? lookahead_min_ : 2.0f;
    const float pred = rr + (rs - rr) * expf(-t_min);   // tau = 60 s: t in minutes is directly the exponent
    const bool wachten = (now - stop_wish_ms_) < stop_delay_ms_;
    const bool terug_nodig = pred < sp - band;
    if (wachten || terug_nodig) {
      if (!stop_hold_) {
        ESP_LOGI(TAG, "AUTO: stop postponed (%s): supply %.1f/%.1f, predicted after stop %.1f",
                 terug_nodig ? "would be needed again immediately" : "stop delay first", rs, sp, pred);
        stop_hold_ = true;
      }
      dir = 0;
    } else {
      stop_hold_ = false;
    }
  }
  if (!wil_stop) {
    stop_wish_ms_ = 0;
    stop_hold_ = false;
  }

  // ── Still warming up? Then do not step up ──
  // If the supply is still rising considerably, the previous step is still taking effect; waiting
  // prevents it from stepping up while the water is simply still warming.
  // When cooling, falling counts as "rising".
  if (dir > 0 && stand_ > 0 && rise_limit_ > 0.0f) {
    float rise = get_rise_();
    if (cooling_ && !std::isnan(rise))
      rise = -rise;
    if (!std::isnan(rise) && rise > rise_limit_) {
      if (!rise_holding_)
        ESP_LOGI(TAG, "AUTO: supply still changing %.1f in 5 min (limit %.1f) - holding stage %u",
                 rise, rise_limit_, stand_);
      rise_holding_ = true;
      dir = 0;
    } else {
      rise_holding_ = false;
    }
  } else {
    rise_holding_ = false;
  }

  // ── Step modulation: max ±1 stage per step_interval (not full throttle) ──
  // Exception: from standstill to stage 1 it waits at most 60 s.
  // With a long interval (e.g. 300 s) it would otherwise stay stopped for up to five minutes
  // while there is demand; the calm build-up afterwards still
  // applies. Those 60 s after the previous step prevent it from starting up again
  // right after switching off.
  uint32_t wacht = (stand_ == 0 && dir > 0 && step_interval_ms_ > 60000) ? 60000 : step_interval_ms_;
  // Down may be faster, but never to 0: stopping keeps the normal interval.
  if (dir < 0 && stand_ > 1 && down_interval_ms_ > 0 && down_interval_ms_ < wacht)
    wacht = down_interval_ms_;
  if (dir != 0 && (now - last_step_ms_) >= wacht) {
    int ns = clamp((int) stand_ + dir, 0, (int) max_stand_);
    if ((uint8_t) ns != stand_) {
      stand_ = (uint8_t) ns;
      last_step_ms_ = now;
      if (std::isnan(sp))
        ESP_LOGI(TAG, "AUTO: no demand (%s) -> stage %u", reason_, stand_);
      else if (vroeg_remmen)
        ESP_LOGI(TAG, "AUTO: supply %.1f/%.1f rising %.1f/min, expected %.1f in %.1f min -> stepping back early to %u",
                 rs, sp, get_rate_(), rs + get_rate_() * lookahead_min_, lookahead_min_, stand_);
      else
        ESP_LOGI(TAG, "AUTO: supply %.1f/%.1f (error %.1f) dT %.1f -> stage %u", rs, sp, err, dt, stand_);
    }
  }
  // Ceiling lowered from HA: go directly below it, not step by step.
  if (stand_ > max_stand_)
    stand_ = max_stand_;
}

// ═══════════════════════════════════════════════════════════════
//  Setters - only log when something actually changes. Automations
//  often set these again every minute; otherwise the one line that
//  matters drowns among hundreds in which nothing happens.
// ═══════════════════════════════════════════════════════════════
void ChofuWP::set_setpoint(float v) {
  const float nieuw = clamp(v, setpoint_min_, setpoint_max_);
  if (fabsf(nieuw - setpoint_) < 0.05f)
    return;
  setpoint_ = nieuw;
  ESP_LOGI(TAG, "Supply setpoint: %.1f°C", setpoint_);
}

void ChofuWP::set_cooling(bool c) {
  if (c == cooling_)
    return;
  cooling_ = c;
  stand_ = 0;          // Always switch via stage 0, never reverse on the fly
  last_step_ms_ = 0;   // May then build up again immediately
  sp_eff_ = NAN;       // The ramping starts again, now in the other direction
  ESP_LOGI(TAG, "Operating mode: %s", c ? "COOLING" : "heating");
}

void ChofuWP::set_system_on(bool on) {
  if (on == system_on_)
    return;
  system_on_ = on;
  ESP_LOGI(TAG, "System: %s", on ? "ON" : "OFF");
}

void ChofuWP::set_mode(Mode m) {
  if (m == mode_)
    return;
  mode_ = m;
  room_i_ = 0.0f;      // The trim starts over at every mode change
  room_i_ms_ = 0;
  room_demand_ = true;
  static const char *const NAMEN[] = {"auto (fixed setpoint)", "heating curve", "room temperature", "manual"};
  ESP_LOGI(TAG, "Mode: %s", NAMEN[m]);
}

void ChofuWP::set_room_temp(float v) {
  room_temp_ = v;
  room_ms_ = millis();
  if (room_ms_ == 0)   // 0 means "never set"
    room_ms_ = 1;
}

void ChofuWP::set_manual_stand(int s) {
  manual_stand_ = clamp(s, 0, (int) max_stand_);
  // Without this message a stage that was requested too high is invisible: the slider
  // allows it, the code clips it and the heat pump neatly reports the lower
  // stage back - which is easy to read as "the unit cannot go higher".
  if (s != (int) manual_stand_)
    ESP_LOGW(TAG, "Stage %d requested but limited to %u (Maximum stage).", s, manual_stand_);
}

}  // namespace chofu_wp
}  // namespace esphome

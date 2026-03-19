# -*- coding: utf-8 -*-
import time as _time
import numpy as np

from robot_cmd import motors_set


# fallback: если у вас они импортируются из общего модуля — можно убрать
MODE_MARKERS = "markers"
MODE_ROAD    = "road"
MODE_TRAJ    = "traj"


class RoadFollowController:
    def __init__(self, state):
        self.state = state

    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    @staticmethod
    def _lp(prev, target, dt, tau):
        tau = max(1e-4, float(tau))
        a = 1.0 - float(np.exp(-float(dt) / tau))
        return float(prev + a * (float(target) - float(prev)))

    def update(self, road_bin: np.ndarray) -> None:
        st = self.state

        # =========================================================
        # MODE GATE (ТОЛЬКО ОБЕРТКА):
        # RF работает ТОЛЬКО в режиме дороги
        # =========================================================
        if getattr(st, "nav_mode", MODE_MARKERS) != MODE_ROAD:
            return

        # gating (ваш)
        if not bool(getattr(st, "is_running", False)):
            return
        if bool(getattr(st, "manual_drive", False)):
            return
        # trajectory_mode больше не нужен как режим, но оставлю как дополнительный safety
        if bool(getattr(st, "trajectory_mode", False)):
            return
        if road_bin is None or getattr(road_bin, "size", 0) == 0:
            return

        # IMPORTANT: B не затираем None-ом
        b_pwm = int(getattr(st, "b_pwm", 1500) or 1500)

        if road_bin.dtype != np.bool_ and road_bin.dtype != np.uint8:
            road_bin = road_bin.astype(np.uint8)

        H, W = road_bin.shape[:2]
        if H < 10 or W < 10:
            return

        # dt
        tnow = _time.monotonic()
        last_t = float(getattr(st, "_rf_last_ts", 0.0) or 0.0)
        dt = float(self._clamp(tnow - last_t, 0.001, 0.25)) if last_t > 0 else 0.05
        st._rf_last_ts = tnow
        st._rf_dt = dt

        # params
        max_delta_pwm = int(getattr(st, "max_delta_pwm", 260) or 260)
        max_delta_pwm = int(self._clamp(max_delta_pwm, 10, 800))

        steer_tau = float(getattr(st, "steer_tau", 0.18) or 0.18)
        steer_tau = float(self._clamp(steer_tau, 0.01, 2.0))

        road_center_bias = float(getattr(st, "road_center_bias", 0.0) or 0.0)
        road_center_bias = float(self._clamp(road_center_bias, -1.0, 1.0))
        road_bias = float(getattr(st, "road_bias", 0.0) or 0.0)
        road_bias = float(self._clamp(road_bias, -1.0, 1.0))
        ui_bias = float(self._clamp(road_center_bias + road_bias, -1.0, 1.0))

        base = int(getattr(st, "pwm_base_us", 1700) or 1700)
        base = int(self._clamp(base, 1500, 2000))

        exec_gain = float(getattr(st, "turn_exec_gain", 1.25) or 1.25)
        exec_gain = float(self._clamp(exec_gain, 1.0, 2.0))

        turn_dir = getattr(st, "next_turn_dir", None)
        jdist = getattr(st, "jdist", None)
        adeg = float(getattr(st, "turn_deg", 0.0) or 0.0)
        abs_deg = float(abs(adeg))

        bias_allowed = bool(getattr(st, "turn_bias_allowed", False))
        turn_allowed = bool(getattr(st, "turn_allowed", False))
        execute_turn = bool(getattr(st, "turn_execute", False))

        if getattr(st, "road_state", "no_road") != "on_road":
            st._rf_steer = 0.0
            motors_set(st, 1500, 1500, b_pwm)
            return

        # -------- RF core --------
        img_f = (road_bin.astype(np.uint8) > 0).astype(np.float32)

        dec_y0 = float(getattr(st, "rf_dec_y0", 0.11) or 0.11)
        dec_y1 = float(getattr(st, "rf_dec_y1", 0.55) or 0.55)
        sup_y0 = float(getattr(st, "rf_sup_y0", 0.55) or 0.55)
        sup_y1 = float(getattr(st, "rf_sup_y1", 0.84) or 0.84)

        dec_y0 = max(0.0, min(0.95, dec_y0))
        dec_y1 = max(dec_y0 + 0.02, min(1.0, dec_y1))
        sup_y0 = max(0.0, min(0.98, sup_y0))
        sup_y1 = max(sup_y0 + 0.02, min(1.0, sup_y1))

        y0d = max(0, min(H - 2, int(H * dec_y0)))
        y1d = max(y0d + 1, min(H, int(H * dec_y1)))
        y0s = max(0, min(H - 2, int(H * sup_y0)))
        y1s = max(y0s + 1, min(H, int(H * sup_y1)))

        dec = img_f[y0d:y1d, :]
        sup = img_f[y0s:y1s, :]

        st._rf_support_frac = float(sup.mean()) if sup.size else 0.0

        col_scores = dec.sum(axis=0)
        total = float(col_scores.sum())

        min_total_dec = float(getattr(st, "rf_min_total_dec", 80.0) or 80.0)
        min_total_dec = float(self._clamp(min_total_dec, 1.0, 5000.0))

        st._rf_total = total
        st.rf_dec_fill = float(total / max(1.0, float(dec.size)))

        if total < min_total_dec:
            st._rf_steer = 0.0
            motors_set(st, base, base, b_pwm)
            return

        mx = float(col_scores.max()) if col_scores.size else 0.0
        st._rf_confidence = (col_scores / mx).astype(np.float32) if mx > 1e-6 else col_scores.astype(np.float32)

        temp = float(getattr(st, "rf_scan_temp", 0.12) or 0.12)
        temp = float(self._clamp(temp, 0.02, 0.50))

        m = float(col_scores.max())
        ww = np.exp((col_scores - m) / max(1e-6, temp))
        w_sum = float(ww.sum())
        if w_sum <= 1e-9:
            x_hat = 0.5 * float(W)
        else:
            ww = ww / w_sum
            xs = np.arange(W, dtype=np.float32)
            x_hat = float((ww * xs).sum())

        # center boost
        center_boost = float(getattr(st, "rf_center_boost", 0.30) or 0.30)
        center_boost = float(self._clamp(center_boost, 0.0, 0.95))

        jdist_val = getattr(st, "jdist", None)
        turn_dir_val = getattr(st, "next_turn_dir", None)

        junc_off_m = float(getattr(st, "rf_center_boost_off_jdist_m", 12.0) or 12.0)
        junc_off_m = float(self._clamp(junc_off_m, 2.0, 40.0))

        approaching = False
        if jdist_val is not None:
            jd = float(jdist_val)
            approaching = (jd >= 0.0) and (jd <= junc_off_m)

        if approaching or (turn_dir_val in ("left", "right")):
            center_boost *= float(getattr(st, "rf_center_boost_junc_mul", 0.0) or 0.0)

        x_hat = (1.0 - center_boost) * x_hat + center_boost * (0.5 * float(W))

        offset_raw = float((x_hat - (W / 2.0)) / (W / 2.0))
        offset_raw = float(self._clamp(offset_raw, -1.0, 1.0))

        st._rf_x_center = x_hat
        st._rf_offset_raw = offset_raw

        # =========================================================
        # ДАЛЬШЕ ВАШ КОД БЕЗ ЛОГИЧЕСКИХ ИЗМЕНЕНИЙ
        # (я не меняю вашу mainline policy)
        # =========================================================

        now = _time.monotonic()
        mainline_allowed = bool(getattr(st, "mainline_allowed", True))

        cam_ts = float(getattr(st, "_cam_junc_last_seen_ts", 0.0) or 0.0)
        cam_recent_s = float(getattr(st, "cam_recent_s", 1.2) or 1.2)
        cam_recent = (cam_ts > 0.0) and ((now - cam_ts) <= cam_recent_s)

        jdist_val = getattr(st, "jdist", None)
        jdist_m = float(jdist_val) if jdist_val is not None else 9999.0

        bias_window_m = float(getattr(st, "mainline_bias_m", 6.0) or 6.0)
        exec_window_m = float(getattr(st, "mainline_exec_m", 2.0) or 2.0)

        turn_dir_val = getattr(st, "next_turn_dir", None)
        turn_mode = (turn_dir_val in ("left", "right"))

        offset_main = None

        if bool(getattr(st, "rails_enabled", False)):
            ml = getattr(st, "_main_line", None)
            L = getattr(st, "_rail_mask_L_full", None)
            R = getattr(st, "_rail_mask_R_full", None)
            crop_y0 = int(getattr(st, "_seg_crop_y0", 0) or 0)

            if ml and L and R:
                try:
                    y_top = int(self._clamp(int(ml.get("y_scan_crop", 0)), 0, H - 2))
                    xL_top = float(self._clamp(float(ml.get("xL_crop", 0.0)), 0.0, float(W - 1)))
                    xR_top = float(self._clamp(float(ml.get("xR_crop", float(W - 1))), 0.0, float(W - 1)))

                    xL_bot_full, yL_bot_full = L
                    xR_bot_full, yR_bot_full = R

                    yL_bot = int(self._clamp(int(yL_bot_full) - crop_y0, 0, H - 1))
                    yR_bot = int(self._clamp(int(yR_bot_full) - crop_y0, 0, H - 1))

                    xL_bot = float(self._clamp(float(xL_bot_full), 0.0, float(W - 1)))
                    xR_bot = float(self._clamp(float(xR_bot_full), 0.0, float(W - 1)))

                    y_ref = int(self._clamp(int(0.5 * (y0d + y1d)), 0, H - 1))
                    y_bot = int(max(yL_bot, yR_bot, y_ref, y_top + 1))

                    denom = float(max(1, (y_bot - y_top)))
                    t = float(self._clamp((y_ref - y_top) / denom, 0.0, 1.0))

                    xl = (1.0 - t) * xL_top + t * xL_bot
                    xr = (1.0 - t) * xR_top + t * xR_bot

                    if xr > xl + 5.0:
                        c = 0.5 * (xl + xr)
                        offset_main = float(self._clamp(
                            (c - (W / 2.0)) / (W / 2.0),
                            -1.0, 1.0
                        ))
                except Exception:
                    offset_main = None

        hold_s = float(getattr(st, "mainline_hold_s", 1.0) or 1.0)
        hold_s = float(self._clamp(hold_s, 0.2, 3.0))

        reacq_need = int(getattr(st, "mainline_reacq_need_frames", 2) or 2)
        reacq_need = int(self._clamp(reacq_need, 1, 10))

        reacq_max_abs = float(getattr(st, "mainline_reacq_max_abs", 0.35) or 0.35)
        reacq_max_abs = float(self._clamp(reacq_max_abs, 0.10, 0.95))

        exec_window_m = float(getattr(st, "mainline_exec_m", 2.0) or 2.0)
        exec_window_m = float(self._clamp(exec_window_m, 0.5, 10.0))

        jdist_val = getattr(st, "jdist", None)
        jdist_m = float(jdist_val) if jdist_val is not None else 9999.0
        prev_exec = bool(getattr(st, "_mainline_prev_exec", False))
        cur_exec  = bool(execute_turn)
        st._mainline_prev_exec = cur_exec

        if (not prev_exec) and cur_exec:
            st._exec_start_ts = float(now)
            st.mainline_off_until_ts = float(now + hold_s)

            st._mainline_reacq_cnt = 0
            st._mainline_reacq_active = True

            st._mainline_reacq_deadline_ts = float(
                now + float(getattr(st, "mainline_reacq_timeout_s", 2.0) or 2.0)
            )

            st._rails_off_active = True

        off_until = float(getattr(st, "mainline_off_until_ts", 0.0) or 0.0)
        hold_active = (now < off_until)

        reacq_active = bool(getattr(st, "_mainline_reacq_active", False))
        deadline = float(getattr(st, "_mainline_reacq_deadline_ts", 0.0) or 0.0)

        good_main = False
        if offset_main is not None:
            good_main = (abs(float(offset_main)) <= reacq_max_abs)
        else:
            ml2 = getattr(st, "_main_line", None)
            if ml2:
                try:
                    xL = float(ml2.get("xL_crop", 0.0))
                    xR = float(ml2.get("xR_crop", float(W - 1)))
                    c = 0.5 * (xL + xR)
                    off_ml = (c - (W / 2.0)) / (W / 2.0)
                    good_main = (abs(float(off_ml)) <= reacq_max_abs)
                except Exception:
                    good_main = False

        if hold_active:
            st._mainline_reacq_cnt = 0
        else:
            if reacq_active:
                if good_main:
                    st._mainline_reacq_cnt = int(getattr(st, "_mainline_reacq_cnt", 0) or 0) + 1
                else:
                    st._mainline_reacq_cnt = 0

                if int(getattr(st, "_mainline_reacq_cnt", 0) or 0) >= reacq_need:
                    st._mainline_reacq_active = False
                    st._mainline_reacq_cnt = 0
                    st._rails_off_active = False
                    st.turn_execute = False
                    execute_turn = False

        if reacq_active and (deadline > 0.0) and (now >= deadline):
            st._mainline_reacq_active = False
            st._mainline_reacq_cnt = 0
            st._rails_off_active = False
            st.turn_execute = False
            execute_turn = False

        if (not turn_mode) and (not bool(st.turn_execute)):
            st._mainline_reacq_active = False
            st._mainline_reacq_cnt = 0
            st._rails_off_active = False

        mainline_allowed = bool(getattr(st, "mainline_allowed", True))
        if hold_active:
            mainline_allowed = False
        elif bool(getattr(st, "_mainline_reacq_active", False)):
            mainline_allowed = False
        st.mainline_allowed = bool(mainline_allowed)

        w_main = 0.0
        if mainline_allowed and (offset_main is not None):
            if (jdist_m <= bias_window_m) and turn_mode:
                w_main = float(getattr(st, "mainline_bias_w", 0.25) or 0.25)
                w_main = float(self._clamp(w_main, 0.0, 0.8))
            else:
                w_main = float(getattr(st, "mainline_follow_w", 1.0) or 1.0)
                w_main = float(self._clamp(w_main, 0.0, 1.0))

        offset = float(self._clamp(offset_raw + ui_bias, -1.0, 1.0))

        force_mainline = bool(getattr(st, "force_mainline_only", True))
        if (offset_main is not None) and (w_main > 0.0):
            if force_mainline and (w_main >= 0.99):
                offset = float(offset_main)
            else:
                offset = float((1.0 - w_main) * offset + w_main * float(offset_main))

        offset = float(self._clamp(offset, -1.0, 1.0))

        st._rf_offset_main = float(offset_main) if offset_main is not None else None
        st._rf_main_w = float(w_main)

        main_half_w = float(getattr(st, "main_road_half_w", 0.18) or 0.18)
        main_half_w = float(self._clamp(main_half_w, 0.05, 0.45))
        off_lo = float(self._clamp(offset - main_half_w, -1.0, 1.0))
        off_hi = float(self._clamp(offset + main_half_w, -1.0, 1.0))
        st._main_px_crop = (
            int((W * 0.5) + (W * 0.5) * off_lo),
            int((W * 0.5) + (W * 0.5) * off_hi),
        )
        st._main_px_full = st._main_px_crop

        turn_light_target = float(getattr(st, "turn_light_target", 0.10) or 0.10)
        turn_mid_target   = float(getattr(st, "turn_mid_target",   0.23) or 0.23)
        turn_hard_target  = float(getattr(st, "turn_hard_target",  0.65) or 0.65)

        turn_light_deg = float(getattr(st, "turn_light_deg", 30.0) or 30.0)
        turn_hard_deg  = float(getattr(st, "turn_hard_deg", 110.0) or 110.0)

        def bias_target_for(tdir, abs_deg_val):
            if tdir == "left":
                if abs_deg_val < turn_light_deg: return -turn_light_target
                if abs_deg_val < turn_hard_deg:  return -turn_mid_target
                return -turn_hard_target
            if tdir == "right":
                if abs_deg_val < turn_light_deg: return +turn_light_target
                if abs_deg_val < turn_hard_deg:  return +turn_mid_target
                return +turn_hard_target
            return 0.0

        if bias_allowed and turn_allowed and turn_dir in ("left", "right"):
            bt = float(bias_target_for(turn_dir, abs_deg))
            bias_m = float(getattr(st, "turn_bias_m", 8.0) or 8.0)
            jd2 = float(jdist) if jdist is not None else 999.0
            alpha = 1.0 - float(self._clamp(jd2 / max(1e-6, bias_m), 0.0, 1.0))
            if execute_turn:
                alpha = float(self._clamp(alpha + 0.35, 0.0, 1.0))
            st._rf_turn_bias_alpha = float(alpha)
            offset = float((1.0 - alpha) * offset + alpha * bt)
            offset = float(self._clamp(offset, -1.0, 1.0))

        prev = float(getattr(st, "_rf_steer", 0.0) or 0.0)
        steer = self._lp(prev, offset, dt, steer_tau)

        if execute_turn:
            steer = float(self._clamp(steer * exec_gain, -1.0, 1.0))

        steer = float(self._clamp(steer, -1.0, 1.0))
        st._rf_steer = steer

        delta = int(self._clamp(steer * float(max_delta_pwm), -max_delta_pwm, max_delta_pwm))
        l = int(self._clamp(base + delta, 1000, 2000))
        r = int(self._clamp(base - delta, 1000, 2000))

        motors_set(st, l, r, b_pwm)

        st._rf_dec_y0, st._rf_dec_y1 = y0d, y1d
        st._rf_sup_y0, st._rf_sup_y1 = y0s, y1s
        st._rf_offset = offset

        # debug print block (ваш, только без логики изменений)
        try:
            now2 = _time.monotonic()

            jd = getattr(st, "jdist", None)
            jdist_m = float(jd) if jd is not None else 9999.0
            turn_dir = getattr(st, "next_turn_dir", None)
            turn_mode = (turn_dir in ("left", "right"))

            snap_m = float(getattr(st, "snap_before_m", 8.0) or 8.0)
            exec_m = float(getattr(st, "turn_exec_m", 2.0) or 2.0)

            off_until = float(getattr(st, "mainline_off_until_ts", 0.0) or 0.0)
            allowed = bool(getattr(st, "mainline_allowed", True)) and (now2 >= off_until) and (not bool(getattr(st, "_mainline_reacq_active", False)))
            exec_flag = bool(getattr(st, "turn_execute", False))

            stage = "STRAIGHT"
            if turn_mode:
                if exec_flag or (not allowed):
                    stage = "EXEC"
                elif jdist_m <= snap_m:
                    stage = "APPROACH"
                else:
                    stage = "FAR"

            prev_stage = getattr(st, "_rf_stage_prev", None)
            if prev_stage != stage:
                st._rf_stage_prev = stage

                jd_txt = "—" if jd is None else f"{jdist_m:.2f}m"
                w_main_dbg = float(getattr(st, "_rf_main_w", 0.0) or 0.0)

                main_txt = "—"
                if getattr(st, "_rf_offset_main", None) is not None:
                    main_txt = f"{float(st._rf_offset_main):+.2f}"

                rails_off = int(bool(getattr(st, "_rails_off_active", False)))
                bias_a = float(getattr(st, "_rf_turn_bias_alpha", 0.0) or 0.0)

                print(
                    f"[RF] stage={stage} dir={turn_dir} jdist={jd_txt} rails_off={rails_off} "
                    f"steer={st._rf_steer:+.2f} off_raw={st._rf_offset_raw:+.2f} "
                    f"main={main_txt} w_main={w_main_dbg:.2f} biasA={bias_a:.2f}",
                    flush=True
                )
        except Exception:
            pass
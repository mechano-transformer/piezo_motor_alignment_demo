"""
自動軸振り分けプログラム（auto_divisioner.py）

仕様:
  1. 現在の角度を読み取る
  2. Axis 1 を +3000 パルス駆動
  3. 角度を読み取り、角度差の大きい軸が axis1 の対応軸。差 < 0 なら reverse axis1
  4. Axis 2 を +3000 パルス駆動
  5. 角度を読み取り、3 と違う軸が axis2。差 < 0 なら reverse axis2
  6. Axis1 は Y 軸にする必要がある。axis1 が X なら swap x/y axes を有効化
  7. 検証: axis1 を +3000 駆動して Y 軸の角度差 > 0 を確認
  8. 検証: axis2 を +3000 駆動して X 軸の角度差 > 0 を確認
  9. 7 or 8 が失敗ならエラー表示。結果に関係なく終了

ポイント:
  - 角度単位は deg
  - 駆動完了を wait_for_stop で確認してから角度を読み取る（フォールバック: 3秒待機）

おまけ:
  - Pulse calibration = 3000 / Δdeg を自動計算
"""
import threading
import time


# 駆動パルス数
DRIVE_PULSES = 3000

# 駆動完了後の読み取り待機時間（wait_for_stop が使えない場合のフォールバック）
SETTLE_DELAY = 3.0


class AutoDivisionerThread(threading.Thread):
    """自動軸振り分けスレッド。

    Args:
        master: ADCGUI インスタンス
        on_complete: 完了時コールバック (success: bool, message: str) -> None
    """

    def __init__(self, master, on_complete=None):
        super().__init__()
        self.daemon = True
        self.master = master
        self.on_complete = on_complete
        self.running = False

    def run(self) -> None:
        self.running = True
        try:
            success, message = self._execute()
        except Exception as e:
            import traceback
            traceback.print_exc()
            success, message = False, f"Unexpected error: {e}"
        finally:
            self.running = False
            if self.on_complete:
                self.master.after(0, lambda: self.on_complete(success, message))

    def _read_angle(self) -> tuple[float, float]:
        """現在の角度を読み取る (x, y)。スムージング設定を反映する。"""
        if self.master.smoothing_enabled:
            return self.master.alnx_smooth, self.master.alny_smooth
        return self.master.alnx, self.master.alny

    def _drive_and_wait(self, axis: int, pulses: int, apply_settings: bool = False) -> bool:
        """指定軸を駆動し、完了を待つ。

        Args:
            axis: 論理軸番号 (1 or 2)
            pulses: 駆動パルス数
            apply_settings: True の場合、swap_axes / reverse_axis の設定を反映して駆動する。
                           検証ステップ (Step 7, 8) では True にする。

        wait_for_stop を試み、失敗した場合は SETTLE_DELAY 秒待機する。
        """
        pamc = self.master.pamc
        physical_axis = axis
        physical_pulses = pulses

        if apply_settings:
            if self.master.swap_axes:
                physical_axis = 2 if axis == 1 else 1
            if physical_axis == 1 and self.master.reverse_axis1:
                physical_pulses = -pulses
            elif physical_axis == 2 and self.master.reverse_axis2:
                physical_pulses = -pulses

        ok = pamc.move_relative(physical_axis, physical_pulses)
        if not ok:
            print(f"[AutoDiv] move_relative axis={physical_axis} pulses={physical_pulses:+d} FAILED")
            return False

        finished = pamc.wait_for_stop(physical_axis, timeout=10.0)
        if not finished:
            print(f"[AutoDiv] wait_for_stop timeout for axis {physical_axis}, waiting {SETTLE_DELAY}s as fallback")
            time.sleep(SETTLE_DELAY)

        # 駆動後の安定化のため少し待つ
        time.sleep(0.5)
        return True

    def _execute(self) -> tuple[bool, str]:
        """自動軸振り分けを実行する。"""
        pamc = self.master.pamc
        if not pamc.is_connected:
            return False, "PAMC not connected"

        if not self.master.worker or not self.master.worker.running:
            return False, "Autocollimator not reading"

        # 単位が deg であることを確認
        if self.master.current_unit != "deg":
            return False, f"Unit must be 'deg', but current unit is '{self.master.current_unit}'"

        # 設定をリセット
        self.master.reverse_axis1 = False
        self.master.reverse_axis2 = False
        self.master.swap_axes = False
        self.master.after(0, lambda: self.master.reverse_axis1_var.set(False))
        self.master.after(0, lambda: self.master.reverse_axis2_var.set(False))
        self.master.after(0, lambda: self.master.swap_axes_var.set(False))

        log = []

        # ── Step 1: 現在の角度を読み取る ──
        time.sleep(0.5)  # 読み取り安定化
        x0, y0 = self._read_angle()
        log.append(f"Step 1: Initial angle: X={x0:.4f}, Y={y0:.4f} deg")
        print(f"[AutoDiv] {log[-1]}")

        # ── Step 2: Axis 1 を +3000 パルス駆動 ──
        log.append(f"Step 2: Driving Axis 1 by +{DRIVE_PULSES} pulses")
        print(f"[AutoDiv] {log[-1]}")
        if not self._drive_and_wait(1, DRIVE_PULSES):
            return False, "Failed to drive Axis 1"

        # ── Step 3: 角度を読み取り、axis1 の対応軸を決定 ──
        x1, y1 = self._read_angle()
        dx1 = x1 - x0
        dy1 = y1 - y0
        log.append(f"Step 3: After Axis 1 drive: X={x1:.4f}, Y={y1:.4f} deg (dX={dx1:.4f}, dY={dy1:.4f})")
        print(f"[AutoDiv] {log[-1]}")

        if abs(dx1) > abs(dy1):
            axis1_maps_to = "x"
            axis1_diff = dx1
        else:
            axis1_maps_to = "y"
            axis1_diff = dy1

        reverse_axis1 = axis1_diff < 0
        log.append(f"Step 3: Axis 1 -> {axis1_maps_to.upper()} axis (diff={axis1_diff:.4f}), reverse={reverse_axis1}")
        print(f"[AutoDiv] {log[-1]}")

        # ── Step 4: Axis 2 を +3000 パルス駆動 ──
        log.append(f"Step 4: Driving Axis 2 by +{DRIVE_PULSES} pulses")
        print(f"[AutoDiv] {log[-1]}")
        if not self._drive_and_wait(2, DRIVE_PULSES):
            return False, "Failed to drive Axis 2"

        # ── Step 5: 角度を読み取り、axis2 の対応軸を決定 ──
        x2, y2 = self._read_angle()
        dx2 = x2 - x1
        dy2 = y2 - y1
        log.append(f"Step 5: After Axis 2 drive: X={x2:.4f}, Y={y2:.4f} deg (dX={dx2:.4f}, dY={dy2:.4f})")
        print(f"[AutoDiv] {log[-1]}")

        # axis2 は axis1 と違う軸
        axis2_maps_to = "y" if axis1_maps_to == "x" else "x"
        axis2_diff = dy2 if axis2_maps_to == "y" else dx2

        reverse_axis2 = axis2_diff < 0
        log.append(f"Step 5: Axis 2 -> {axis2_maps_to.upper()} axis (diff={axis2_diff:.4f}), reverse={reverse_axis2}")
        print(f"[AutoDiv] {log[-1]}")

        # ── Step 6: Axis1 は Y 軸にする必要がある ──
        swap_axes = (axis1_maps_to == "x")
        log.append(f"Step 6: Swap X/Y axes = {swap_axes} (Axis1 maps to {axis1_maps_to.upper()})")
        print(f"[AutoDiv] {log[-1]}")

        # 設定を適用
        self.master.reverse_axis1 = reverse_axis1
        self.master.reverse_axis2 = reverse_axis2
        self.master.swap_axes = swap_axes
        self.master.after(0, lambda: self.master.reverse_axis1_var.set(reverse_axis1))
        self.master.after(0, lambda: self.master.reverse_axis2_var.set(reverse_axis2))
        self.master.after(0, lambda: self.master.swap_axes_var.set(swap_axes))

        # ── Step 7: 検証 - Axis 1 を +3000 駆動して Y 軸の角度差 > 0 を確認 ──
        time.sleep(0.5)
        x_before, y_before = self._read_angle()
        log.append(f"Step 7: Verification - Driving Axis 1 by +{DRIVE_PULSES} pulses")
        print(f"[AutoDiv] {log[-1]}")
        if not self._drive_and_wait(1, DRIVE_PULSES, apply_settings=True):
            return False, "Failed to drive Axis 1 (verification)"

        x_after, y_after = self._read_angle()
        dy_verify = y_after - y_before
        step7_ok = dy_verify > 0
        log.append(f"Step 7: Y diff = {dy_verify:.4f} deg -> {'OK' if step7_ok else 'FAIL'}")
        print(f"[AutoDiv] {log[-1]}")

        # ── Step 8: 検証 - Axis 2 を +3000 駆動して X 軸の角度差 > 0 を確認 ──
        time.sleep(0.5)
        x_before, y_before = self._read_angle()
        log.append(f"Step 8: Verification - Driving Axis 2 by +{DRIVE_PULSES} pulses")
        print(f"[AutoDiv] {log[-1]}")
        if not self._drive_and_wait(2, DRIVE_PULSES, apply_settings=True):
            return False, "Failed to drive Axis 2 (verification)"

        x_after, y_after = self._read_angle()
        dx_verify = x_after - x_before
        step8_ok = dx_verify > 0
        log.append(f"Step 8: X diff = {dx_verify:.4f} deg -> {'OK' if step8_ok else 'FAIL'}")
        print(f"[AutoDiv] {log[-1]}")

        # ── おまけ: Pulse calibration 自動計算・自動設定 ──
        # Step 7 の Y 差分と Step 8 の X 差分の平均を使用
        cal_values = []
        if abs(dy_verify) > 0.001:
            cal_y = DRIVE_PULSES / abs(dy_verify)
            cal_values.append(cal_y)
            log.append(f"Pulse calibration (Y/Axis1): {cal_y:.1f} pulses/deg")
            print(f"[AutoDiv] {log[-1]}")
        if abs(dx_verify) > 0.001:
            cal_x = DRIVE_PULSES / abs(dx_verify)
            cal_values.append(cal_x)
            log.append(f"Pulse calibration (X/Axis2): {cal_x:.1f} pulses/deg")
            print(f"[AutoDiv] {log[-1]}")

        if cal_values:
            avg_cal = sum(cal_values) / len(cal_values)
            cal_int = str(int(round(avg_cal)))
            self.master.after(0, lambda: self.master.ADC_pulses_per_unit_var.set(cal_int))
            log.append(f"Pulse calibration applied to ADC: {cal_int} pulses/deg")
            print(f"[AutoDiv] {log[-1]}")

        # ── Step 9: 結果判定 ──
        if step7_ok and step8_ok:
            summary = (
                f"Auto axis assignment completed successfully.\n\n"
                f"Reverse Axis 1: {reverse_axis1}\n"
                f"Reverse Axis 2: {reverse_axis2}\n"
                f"Swap X/Y Axes: {swap_axes}\n"
            )
            if abs(dy_verify) > 0.001:
                summary += f"\nPulse Cal (Axis1/Y): {DRIVE_PULSES / abs(dy_verify):.1f} pulses/deg"
            if abs(dx_verify) > 0.001:
                summary += f"\nPulse Cal (Axis2/X): {DRIVE_PULSES / abs(dx_verify):.1f} pulses/deg"
            return True, summary
        else:
            errors = []
            if not step7_ok:
                errors.append(f"Step 7 FAILED: Axis 1 +{DRIVE_PULSES} pulses -> Y diff = {dy_verify:.4f} (expected > 0)")
            if not step8_ok:
                errors.append(f"Step 8 FAILED: Axis 2 +{DRIVE_PULSES} pulses -> X diff = {dx_verify:.4f} (expected > 0)")
            error_msg = (
                "Verification failed!\n\n"
                + "\n".join(errors)
                + f"\n\nSettings applied:\n"
                f"  Reverse Axis 1: {reverse_axis1}\n"
                f"  Reverse Axis 2: {reverse_axis2}\n"
                f"  Swap X/Y Axes: {swap_axes}"
            )
            return False, error_msg

    def stop(self) -> None:
        self.running = False

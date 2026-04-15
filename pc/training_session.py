#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
训练会话模块
管理单次训练的状态机、实时数据缓存、步数统计和文件存储。

状态机：idle → running ↔ paused → completed
"""

import os
import json
import time
from datetime import datetime


class TrainingSession:
    """
    代表一次训练会话（主动或被动）。

    生命周期：
        session = TrainingSession()
        session.start(patient, training_type)
        # 运动过程中持续调用：
        session.add_data(data_dict)
        # 可选暂停/恢复：
        session.pause() / session.resume()
        # 训练结束：
        filepath = session.stop()
    """

    STATE_IDLE = "idle"
    STATE_RUNNING = "running"
    STATE_PAUSED = "paused"
    STATE_COMPLETED = "completed"

    def __init__(self):
        self.state = self.STATE_IDLE
        self.patient: dict | None = None
        self.training_type: str = "active"  # "active" | "passive"

        # 时间统计
        self._start_wall: float = 0.0       # wall clock 训练开始时刻
        self._active_start: float = 0.0     # 当前运行段开始时刻
        self._active_accumulated: float = 0.0  # 已累计的有效训练秒数（不含当前暂停）

        # 暂停记录
        self._pauses: list = []             # [{"paused_at": str, "resumed_at": str}, ...]
        self._pause_start: float = 0.0

        # 数据缓存
        self.realtime_data: list = []       # 全量实时数据点列表

        # 步数统计（通过 ph 字段的跳变检测）
        self.step_count: int = 0
        self._last_ph: int | None = None    # 上一帧的步态相位

        # 文件路径
        self._patient_dir: str = ""
        self._temp_filepath: str = ""
        self._final_filepath: str = ""
        self._last_flush_time: float = 0.0

        # 时间戳字符串（用于文件命名）
        self._start_time_str: str = ""
        self._end_time_str: str = ""

    # ------------------------------------------------------------------
    # 状态机控制
    # ------------------------------------------------------------------

    def start(self, patient: dict, training_type: str, patient_dir: str):
        """
        开始训练会话。
        :param patient: 患者信息 dict
        :param training_type: "active" 或 "passive"
        :param patient_dir: 患者数据目录路径
        """
        if self.state != self.STATE_IDLE:
            raise RuntimeError("当前会话未处于空闲状态，无法开始新训练")

        self.patient = patient
        self.training_type = training_type
        self._patient_dir = patient_dir

        now = datetime.now()
        self._start_time_str = now.strftime("%Y-%m-%d %H:%M:%S")
        ts = now.strftime("%y-%m-%d-%H-%M")
        name = patient.get("name", "unknown")

        # 文件路径
        self._temp_filepath = os.path.join(patient_dir, f"{name}-{ts}.tmp.json")
        self._final_filepath = os.path.join(patient_dir, f"{name}-{ts}.json")

        # 重置统计
        self.realtime_data = []
        self.step_count = 0
        self._last_ph = None
        self._pauses = []
        self._active_accumulated = 0.0

        now_ts = time.time()
        self._start_wall = now_ts
        self._active_start = now_ts
        self._last_flush_time = now_ts

        self.state = self.STATE_RUNNING

    def pause(self):
        """暂停训练，停止累计有效时长。"""
        if self.state != self.STATE_RUNNING:
            return
        now_ts = time.time()
        self._active_accumulated += now_ts - self._active_start
        self._pause_start = now_ts
        self._pauses.append({"paused_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                              "resumed_at": ""})
        self.state = self.STATE_PAUSED

    def resume(self):
        """恢复训练，继续累计有效时长。"""
        if self.state != self.STATE_PAUSED:
            return
        now_ts = time.time()
        self._active_start = now_ts
        if self._pauses:
            self._pauses[-1]["resumed_at"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.state = self.STATE_RUNNING

    def stop(self) -> str:
        """
        结束训练，保存最终文件，删除临时文件。
        :return: 最终文件的绝对路径
        """
        if self.state not in (self.STATE_RUNNING, self.STATE_PAUSED):
            raise RuntimeError("会话未处于运行或暂停状态")

        if self.state == self.STATE_RUNNING:
            self._active_accumulated += time.time() - self._active_start
        if self.state == self.STATE_PAUSED and self._pauses:
            self._pauses[-1]["resumed_at"] = "（训练结束时中止）"

        self._end_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.state = self.STATE_COMPLETED

        self._save_final()
        self._delete_temp()
        return os.path.abspath(self._final_filepath)

    def reset(self):
        """将会话重置为空闲状态，供下次训练复用。"""
        self.state = self.STATE_IDLE
        self.patient = None
        self.realtime_data = []
        self.step_count = 0
        self._last_ph = None
        self._pauses = []
        self._active_accumulated = 0.0
        self._temp_filepath = ""
        self._final_filepath = ""

    # ------------------------------------------------------------------
    # 数据录入
    # ------------------------------------------------------------------

    def add_data(self, data_dict: dict):
        """
        追加一帧实时数据。仅在 STATE_RUNNING 时有效。
        同时检测步态相位跳变以计步。
        """
        if self.state != self.STATE_RUNNING:
            return

        self.realtime_data.append(data_dict)
        self._detect_step(data_dict)

    def _detect_step(self, data_dict: dict):
        """
        检测步数：ph 字段从非零（摆动相）→ 0（支撑相）时计为一步。
        同时兼容 ph4 字段（0=支撑，其余=摆动）。
        """
        ph = data_dict.get("ph")
        if ph is None:
            ph = data_dict.get("ph4")
        if ph is None:
            return

        ph = int(ph)
        if self._last_ph is not None and self._last_ph != 0 and ph == 0:
            self.step_count += 1
        self._last_ph = ph

    # ------------------------------------------------------------------
    # 文件 I/O
    # ------------------------------------------------------------------

    def flush_temp(self):
        """
        将当前缓存写入临时文件（每 5 秒由 GUI 定时调用）。
        STATE_RUNNING 或 STATE_PAUSED 时均可执行。
        """
        if self.state not in (self.STATE_RUNNING, self.STATE_PAUSED):
            return
        now_ts = time.time()
        if now_ts - self._last_flush_time < 5.0:
            return
        self._last_flush_time = now_ts
        self._write_json(self._temp_filepath, self._build_payload(is_temp=True))

    def _save_final(self):
        """写入最终完整训练记录。"""
        self._write_json(self._final_filepath, self._build_payload(is_temp=False))

    def _delete_temp(self):
        """删除临时文件（若存在）。"""
        if self._temp_filepath and os.path.isfile(self._temp_filepath):
            try:
                os.remove(self._temp_filepath)
            except Exception:
                pass

    def _build_payload(self, is_temp: bool) -> dict:
        """构建要写入文件的完整数据结构。"""
        active_s = self.get_active_duration()
        return {
            "patient": self.patient,
            "training_type": self.training_type,
            "start_time": self._start_time_str,
            "end_time": self._end_time_str if not is_temp else "",
            "active_duration_s": round(active_s, 1),
            "step_count": self.step_count,
            "pauses": self._pauses,
            "realtime_data": self.realtime_data,
        }

    @staticmethod
    def _write_json(filepath: str, payload: dict):
        """安全写入 JSON 文件（先写临时再重命名，防止写入中断损坏）。"""
        tmp = filepath + ".writing"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False)
            os.replace(tmp, filepath)
        except Exception as e:
            print(f"[TrainingSession] 写文件失败 {filepath}: {e}")

    # ------------------------------------------------------------------
    # 统计查询
    # ------------------------------------------------------------------

    def get_elapsed_seconds(self) -> float:
        """返回从训练开始到现在的总经过秒数（含暂停时间）。"""
        if self.state == self.STATE_IDLE:
            return 0.0
        return time.time() - self._start_wall

    def get_active_duration(self) -> float:
        """返回有效训练时长（不含暂停时间）秒数。"""
        if self.state == self.STATE_RUNNING:
            return self._active_accumulated + (time.time() - self._active_start)
        return self._active_accumulated

    def get_stats(self) -> dict:
        """返回当前训练统计摘要。"""
        return {
            "state": self.state,
            "elapsed_s": self.get_elapsed_seconds(),
            "active_duration_s": self.get_active_duration(),
            "step_count": self.step_count,
        }

    @staticmethod
    def format_duration(seconds: float) -> str:
        """将秒数格式化为 HH:MM:SS 字符串。"""
        s = int(seconds)
        h = s // 3600
        m = (s % 3600) // 60
        sec = s % 60
        return f"{h:02d}:{m:02d}:{sec:02d}"

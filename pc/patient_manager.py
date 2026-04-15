#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
患者信息管理模块
负责患者数据的 CRUD 操作、目录管理和训练记录索引。
"""

import os
import json
from datetime import date, datetime


PATIENTS_BASE_DIR = "patients"


class PatientManager:
    """管理患者信息和训练记录的持久化存储。"""

    def __init__(self, base_dir: str = PATIENTS_BASE_DIR):
        self.base_dir = base_dir
        os.makedirs(self.base_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # ID 生成
    # ------------------------------------------------------------------

    def generate_id(self) -> str:
        """扫描现有患者目录，生成下一个三位零填充编号（如 '003'）。"""
        existing_ids = []
        if os.path.isdir(self.base_dir):
            for entry in os.listdir(self.base_dir):
                if os.path.isdir(os.path.join(self.base_dir, entry)):
                    # 目录名格式：{id}_{name}，取下划线前的部分
                    parts = entry.split("_", 1)
                    if parts[0].isdigit():
                        existing_ids.append(int(parts[0]))
        next_id = max(existing_ids, default=0) + 1
        return f"{next_id:03d}"

    # ------------------------------------------------------------------
    # CRUD
    # ------------------------------------------------------------------

    def create_patient(self, name: str, birth_date: str,
                       height: float, weight: float) -> dict:
        """
        创建新患者记录。
        :param name: 患者姓名
        :param birth_date: 出生日期字符串 'YYYY-MM-DD'
        :param height: 身高 (cm)
        :param weight: 体重 (kg)
        :return: 患者信息 dict
        """
        patient_id = self.generate_id()
        patient = {
            "id": patient_id,
            "name": name,
            "birth_date": birth_date,
            "height": height,
            "weight": weight,
            "created_at": date.today().isoformat(),
        }
        patient_dir = self.get_patient_dir(patient_id, name)
        os.makedirs(patient_dir, exist_ok=True)
        info_path = os.path.join(patient_dir, "patient.json")
        with open(info_path, "w", encoding="utf-8") as f:
            json.dump(patient, f, ensure_ascii=False, indent=2)
        return patient

    def get_all_patients(self) -> list:
        """
        读取所有患者信息，按编号升序排列。
        :return: list of patient dict
        """
        patients = []
        if not os.path.isdir(self.base_dir):
            return patients
        for entry in sorted(os.listdir(self.base_dir)):
            entry_path = os.path.join(self.base_dir, entry)
            if not os.path.isdir(entry_path):
                continue
            info_path = os.path.join(entry_path, "patient.json")
            if os.path.isfile(info_path):
                try:
                    with open(info_path, "r", encoding="utf-8") as f:
                        patients.append(json.load(f))
                except Exception:
                    pass
        return patients

    def get_patient_by_id(self, patient_id: str) -> dict | None:
        """根据编号获取患者信息。"""
        for p in self.get_all_patients():
            if p.get("id") == patient_id:
                return p
        return None

    # ------------------------------------------------------------------
    # 目录路径
    # ------------------------------------------------------------------

    def get_patient_dir(self, patient_id: str, name: str = "") -> str:
        """
        返回患者数据目录路径（不要求已存在）。
        若 name 为空则自动从已存在目录推断。
        """
        if name:
            return os.path.join(self.base_dir, f"{patient_id}_{name}")
        # 从磁盘查找
        if os.path.isdir(self.base_dir):
            for entry in os.listdir(self.base_dir):
                if entry.startswith(f"{patient_id}_"):
                    return os.path.join(self.base_dir, entry)
        return os.path.join(self.base_dir, patient_id)

    # ------------------------------------------------------------------
    # 训练记录
    # ------------------------------------------------------------------

    def get_training_records(self, patient: dict) -> list:
        """
        列出指定患者的所有已完成训练记录（.json，排除 .tmp.json），
        按文件修改时间降序排列（最新在前）。
        :return: list of absolute file paths
        """
        patient_dir = self.get_patient_dir(patient["id"], patient.get("name", ""))
        if not os.path.isdir(patient_dir):
            return []
        records = []
        for fname in os.listdir(patient_dir):
            if fname == "patient.json":
                continue
            if fname.endswith(".tmp.json"):
                continue
            if fname.endswith(".json"):
                fpath = os.path.join(patient_dir, fname)
                records.append(fpath)
        records.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return records

    def load_training_record(self, filepath: str) -> dict:
        """
        读取并返回训练记录完整内容。
        :raises Exception: 文件不存在或解析失败时
        """
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"记录文件不存在: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            return json.load(f)

    # ------------------------------------------------------------------
    # 辅助：计算年龄
    # ------------------------------------------------------------------

    @staticmethod
    def calc_age(birth_date_str: str) -> int:
        """根据出生日期字符串 'YYYY-MM-DD' 计算周岁。"""
        try:
            bd = date.fromisoformat(birth_date_str)
            today = date.today()
            return today.year - bd.year - (
                (today.month, today.day) < (bd.month, bd.day)
            )
        except Exception:
            return 0

    @staticmethod
    def format_record_label(filepath: str, record: dict | None = None) -> str:
        """
        生成训练记录在列表中的显示文字。
        优先用 record dict，否则仅用文件名。
        """
        fname = os.path.basename(filepath)
        if record is None:
            return fname
        training_type = "主动" if record.get("training_type") == "active" else "被动"
        start_time = record.get("start_time", "")
        active_s = record.get("active_duration_s", 0)
        minutes = int(active_s) // 60
        seconds = int(active_s) % 60
        steps = record.get("step_count", 0)
        return f"{start_time[:16]}  {training_type}  {minutes}分{seconds:02d}秒  {steps}步"

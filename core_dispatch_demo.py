from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
import re
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any, Dict, Iterable, List, Optional, Tuple

from PIL import Image, ImageDraw, ImageFont


DEFAULT_INPUT = "/Users/levin/Documents/泰州无人机调度资料/无人机调度结构.json"
DEFAULT_OUTPUT = "core_demo_result.json"

CONFIGURABLE_PARAMS = {
    # 是否启用内置全局分配优化；关闭后回退到原有逐对象贪心指派。
    "P_ENABLE_ASSIGNMENT_OPTIMIZER": True,
    # 任务规模超过该值时使用贪心近似，避免在线调度出现过长求解时间。
    "P_ASSIGNMENT_EXACT_LIMIT": 12,
    # 分支定界最多搜索节点数；达到上限后保留当前最好解并停止搜索。
    "P_ASSIGNMENT_NODE_LIMIT": 200000,
    # 是否启用固定航线下的最少架次分段优化。
    "P_ENABLE_SORTIE_OPTIMIZER": True,
    # 是否按方案目标优化同一无人机上独立架次的执行顺序。
    "P_ENABLE_SCHEDULE_OPTIMIZER": True,
    # P001: 周期任务风力等级阈值；业务JSON只提供天气文本，阈值由算法配置。
    "P001_wind_periodic_max_level": 4,
    # P001: 机场开启自检时，airport.wind_speed 的最大允许风速，单位 m/s。
    "P001_self_check_max_wind_mps": 8.0,
    # P002: 周期任务遇到这些降雨等级时禁飞。
    "P002_periodic_forbid_weather": ["中雨", "大雨", "暴雨"],
    # P002: 遇到这些降雨等级时允许执行但进入预警。
    "P002_warn_weather": ["小雨"],
    # P002: 机场开启自检时，airport.rainfall 换算成 mm/h 后的最大允许雨量。
    "P002_self_check_max_rainfall_mm_h": 2.5,
    # P003: 能见度阈值，当前JSON缺能见度字段，暂只用于规则审计提示。
    "P003_visibility_min_m": 500,
    # P004: 温度当前仅记录，不做硬拦截；这里配置输出说明。
    "P004_temperature_note": "平台未确认阈值，当前仅记录温度",
    # P005: 天气文本命中这些关键词时禁飞。
    "P005_thunderstorm_keywords": ["雷", "雷暴"],
    # P006: 临时/立即工单起飞最低电量百分比。
    "P006_min_battery_temp_pct": 60,
    # P006: 周期工单起飞最低电量百分比。
    "P006_min_battery_periodic_pct": 85,
    # P008: 机场状态等于该值时视为可用。
    "P008_ready_airport_status": 0,
    # P008/P018: 无人机状态等于该值时视为待机可用。
    "P008_ready_drone_status": -1,
    # P009: 单机日出勤上限；None表示不限制。
    "P009_daily_limit": None,
    # P010: 安全冗余比例，安全冗余=max(飞行时长*比例, 固定值)。
    "P010_safety_ratio": 0.0,
    # P011: 安全冗余固定下限，单位分钟。
    "P011_safety_fixed_min": 3,
    # P012: 工单接入后最早开始准备的响应时间，单位分钟。
    "P012_response_min": 3,
    # P013: 允许抢占的来单等级。
    "P013_preempt_incoming_level": 1,
    # P013: 当前任务进度达到该百分比后不再建议抢占。
    "P013_preempt_progress_guard_pct": 90,
    # P014: 不可抢占任务类型白名单；当前JSON未提供任务类型，默认空。
    "P014_no_preempt_task_types": [],
    # P015: 自动推荐模式等级阈值，当前用于输出模式说明。
    "P015_auto_mode_level": 1,
    # P028: True表示协同拆分只按航线航点评估。
    "P028_route_only": True,
    # P029: 作业对象数量达到该值时触发多机/多机场协同评估。
    "P029_multi_object_trigger": 2,
    # P037: JSON缺失drone.battery_life时使用的默认满电续航，单位分钟。
    "P037_default_battery_life_min": 30,
    # P038: JSON缺失drone.drone_vertical时使用的默认飞行速度，单位 m/s。
    "P038_default_speed_mps": 15,
    # P039: 单个航点默认作业时长，单位秒。
    "P039_work_sec_per_waypoint": 3,
    # P040: 起飞前准备时长，单位分钟。
    "P040_prepare_min": 1,
    # P041: 是否启用 airport.inspection_radius 做覆盖预筛。
    "P041_use_airport_radius": True,
    # P042: 顺路合并距离阈值；None表示暂不做顺路合并。
    "P042_merge_distance_km": None,
    # P043: JSON缺失drone.charging_duration时使用的默认满充/换电时长，单位分钟。
    "P043_default_charge_min": 30,
    # P018: 充电中无人机剩余恢复时间不超过该值时可视为可用，单位分钟。
    "P018_charging_available_max_min": 10,
    # 电量预警线：剩余电量小于等于该值时标记为预警。
    "battery_warn_pct": 40,
    # 电量接力线：剩余电量小于等于该值时触发接力偏好。
    "battery_relay_pct": 30,
    # 电量返航线：剩余电量小于等于该值时判定返航风险。
    "battery_return_pct": 20,
}

PARAMS = CONFIGURABLE_PARAMS.copy()

SCHEME_INFO = {
    "balanced": {
        "label": "均衡方案",
        "description": "默认方案：优先选覆盖范围内最近机场，触发30%接力阈值时拆接力。",
    },
    "time_first": {
        "label": "时效优先方案",
        "description": "偏向更少接力和更短完成时间，但必须守住返航线。",
    },
    "energy_first": {
        "label": "电量稳健方案",
        "description": "偏向电量余量更安全，尽量保持更高预警线。",
    },
}

def scheme_max_use_ratio(scheme_name: str) -> float:
    if scheme_name == "time_first":
        return (100 - PARAMS["battery_return_pct"]) / 100
    if scheme_name == "energy_first":
        return (100 - PARAMS["battery_warn_pct"]) / 100
    return (100 - PARAMS["battery_relay_pct"]) / 100


@dataclass
class Drone:
    drone_id: str
    name: str
    status: int
    battery_pct: int
    speed_mps: float
    battery_life_min: float
    charging_duration_min: float
    charge_remaining_min: Optional[float] = None
    status_raw: Any = None


@dataclass
class Airport:
    uid: str
    name: str
    lon: float
    lat: float
    radius_m: float
    status: int
    need_check: int
    cross_railway: int
    wind_speed_mps: Optional[float]
    rainfall_mm_min: Optional[float]
    current_task_id: str
    task_priority: Optional[int]
    task_progress: float
    drone: Drone
    weather: List[Dict[str, Any]]


@dataclass
class TargetPoint:
    group_id: str
    group_name: str
    name: str
    lon: float
    lat: float
    altitude: float
    route_points: List[Dict[str, Any]]


@dataclass
class WorkSegment:
    group_id: str
    group_name: str
    airport_uid: str
    route_points: List[Dict[str, Any]]
    full_waypoint_count: int = 0
    segment_index: int = 1
    segment_count: int = 1

    @property
    def name(self) -> str:
        return self.group_name

    @property
    def lon(self) -> float:
        return sum(float(p.get("lon")) for p in self.route_points) / len(self.route_points)

    @property
    def lat(self) -> float:
        return sum(float(p.get("lat")) for p in self.route_points) / len(self.route_points)


def strip_json_comments(text: str) -> str:
    """Remove // comments while preserving // inside quoted strings."""
    out: List[str] = []
    in_string = False
    escape = False
    i = 0
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if in_string:
            out.append(ch)
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_string = False
            i += 1
            continue
        if ch == '"':
            in_string = True
            out.append(ch)
            i += 1
            continue
        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] not in "\r\n":
                i += 1
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def parse_dt(value: Any) -> Optional[datetime]:
    text = str(value or "").strip()
    if not text:
        return None
    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%dT%H:%M:%S"):
        try:
            return datetime.strptime(text[:19], fmt)
        except ValueError:
            continue
    return None


def fmt_dt(value: Optional[datetime]) -> str:
    return value.strftime("%Y-%m-%d %H:%M:%S") if value else ""


def load_structure(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.loads(strip_json_comments(f.read()))


def apply_runtime_params(data: Dict[str, Any]) -> None:
    incoming = {}
    for key in ("dispatch_params", "baseline_params", "params"):
        value = data.get(key)
        if isinstance(value, dict):
            incoming.update(value)
    work_order_params = data.get("work_order", {}).get("dispatch_params")
    if isinstance(work_order_params, dict):
        incoming.update(work_order_params)

    aliases = {
        "charging_available_max_min": "P018_charging_available_max_min",
        "self_check_max_wind_mps": "P001_self_check_max_wind_mps",
        "self_check_max_rainfall_mm_h": "P002_self_check_max_rainfall_mm_h",
    }
    for source_key, raw_value in incoming.items():
        param_key = aliases.get(source_key, source_key)
        if param_key not in PARAMS:
            continue
        default_value = CONFIGURABLE_PARAMS[param_key]
        if isinstance(default_value, bool):
            if isinstance(raw_value, bool):
                PARAMS[param_key] = raw_value
            elif isinstance(raw_value, str):
                PARAMS[param_key] = raw_value.strip().lower() in ("1", "true", "yes", "y")
        elif isinstance(default_value, int) and not isinstance(default_value, bool):
            value = parse_float_or_none(raw_value)
            if value is not None:
                PARAMS[param_key] = int(value)
        elif isinstance(default_value, float):
            value = parse_float_or_none(raw_value)
            if value is not None:
                PARAMS[param_key] = value
        else:
            PARAMS[param_key] = raw_value


def parse_battery(value: Any) -> int:
    if isinstance(value, str):
        value = value.strip().replace("%", "")
    return int(float(value))


def parse_float_or_none(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def haversine_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def parse_airports(data: Dict[str, Any]) -> List[Airport]:
    airports = []
    airport_weather = list(data.get("airport_weather") or [])
    for raw in data.get("airport", []):
        d = raw.get("drone", {})
        drone = Drone(
            drone_id=str(d.get("drone_id", "")),
            name=str(d.get("drone_name") or d.get("sn") or d.get("drone_id", "")),
            status=int(d.get("drone_status", 999)),
            battery_pct=parse_battery(d.get("drone_battery", 0)),
            speed_mps=float(d.get("drone_vertical", PARAMS["P038_default_speed_mps"]) or PARAMS["P038_default_speed_mps"]),
            battery_life_min=float(d.get("battery_life", PARAMS["P037_default_battery_life_min"]) or PARAMS["P037_default_battery_life_min"]),
            charging_duration_min=float(d.get("charging_duration", PARAMS["P043_default_charge_min"]) or PARAMS["P043_default_charge_min"]),
            charge_remaining_min=parse_float_or_none(d.get("charge_remaining_min") or d.get("charging_remaining_min")),
            status_raw=d.get("drone_status"),
        )
        airports.append(
            Airport(
                uid=str(raw.get("airport_uid", "")),
                name=str(raw.get("airport_name", "")),
                lon=float(raw.get("airport_lon")),
                lat=float(raw.get("airport_lat")),
                radius_m=float(raw.get("inspection_radius", 0) or 0),
                status=int(raw.get("airport_status", 999)),
                need_check=int(raw.get("airport_need_check", 0) or 0),
                cross_railway=int(raw.get("airport_cross_railway", 0) or 0),
                wind_speed_mps=parse_float_or_none(raw.get("wind_speed")),
                rainfall_mm_min=parse_float_or_none(raw.get("rainfall")),
                current_task_id=str(raw.get("current_task_id") or raw.get("current_task") or ""),
                task_priority=(
                    int(raw.get("task_priority"))
                    if raw.get("task_priority") not in (None, "")
                    else None
                ),
                task_progress=float(raw.get("task_progress", 0) or 0),
                drone=drone,
                weather=airport_weather,
            )
        )
    return airports


def parse_targets(data: Dict[str, Any]) -> List[TargetPoint]:
    work_order = data.get("work_order", {})
    points: List[TargetPoint] = []
    for group in work_order.get("woder_order_detail", []):
        group_id = str(group.get("obj_id", ""))
        group_name = str(group.get("obj_name", ""))
        route_points = list(group.get("obj_data", []))
        if not route_points:
            continue
        lon = sum(float(p.get("lon")) for p in route_points) / len(route_points)
        lat = sum(float(p.get("lat")) for p in route_points) / len(route_points)
        altitude = sum(float(p.get("altitude", 0) or 0) for p in route_points) / len(route_points)
        points.append(
            TargetPoint(
                group_id=group_id,
                group_name=group_name,
                name=group_name,
                lon=lon,
                lat=lat,
                altitude=altitude,
                route_points=route_points,
            )
        )
    return points


def is_periodic_order(work_order: Dict[str, Any]) -> bool:
    return int(work_order.get("woker_order_execution", 1) or 1) == 3


def min_takeoff_battery(work_order: Dict[str, Any]) -> int:
    if is_periodic_order(work_order):
        return int(PARAMS["P006_min_battery_periodic_pct"])
    return int(PARAMS["P006_min_battery_temp_pct"])


def parse_wind_level(wind_text: str) -> Optional[int]:
    tokens = wind_text.replace("级", " 级").split()
    for token in tokens:
        try:
            return int(token)
        except ValueError:
            continue
    return None


def parse_temperature_range(text: str) -> Optional[Tuple[float, float]]:
    cleaned = text.replace("℃", "").replace("°", "")
    parts = cleaned.split("~")
    if len(parts) != 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def parse_percent(text: Any) -> Optional[float]:
    if text is None:
        return None
    try:
        return float(str(text).strip().replace("%", ""))
    except ValueError:
        return None


def weather_ok(airport: Airport, date_str: Optional[str], work_order: Dict[str, Any]) -> Tuple[bool, List[str]]:
    if not airport.weather:
        return True, ["无天气数据，按可飞处理"]
    item = None
    if date_str:
        item = next((w for w in airport.weather if w.get("dateStr") == date_str), None)
    item = item or airport.weather[0]
    weather = str(item.get("weather", ""))
    wind = str(item.get("windSpeed", ""))
    temp = str(item.get("externalTemperature", ""))
    humidity = parse_percent(item.get("humidity"))
    wind_level = parse_wind_level(wind)
    wind_mps = airport.wind_speed_mps
    rainfall_mm_h = airport.rainfall_mm_min * 60 if airport.rainfall_mm_min is not None else None
    messages = []
    ok = True
    if any(x in weather for x in PARAMS["P005_thunderstorm_keywords"]):
        ok = False
        messages.append(f"P005 雷暴禁飞：{weather}")
    if is_periodic_order(work_order) and any(x in weather for x in PARAMS["P002_periodic_forbid_weather"]):
        ok = False
        messages.append(f"P002 周期工单降雨禁飞：{weather}")
    elif any(x in weather for x in PARAMS["P002_warn_weather"]):
        messages.append(f"P002 小雨预警：{weather}")
    if is_periodic_order(work_order) and wind_level is not None and wind_level > PARAMS["P001_wind_periodic_max_level"]:
        ok = False
        messages.append(f"P001 周期工单风力{wind_level}级 > {PARAMS['P001_wind_periodic_max_level']}级")
    if airport.need_check:
        if wind_mps is None:
            ok = False
            messages.append("P001 自检风速缺失，无法通过自检")
        elif wind_mps > PARAMS["P001_self_check_max_wind_mps"]:
            ok = False
            messages.append(f"P001 自检风速{wind_mps}m/s > {PARAMS['P001_self_check_max_wind_mps']}m/s")
        else:
            messages.append(f"P001 自检风速{wind_mps}m/s <= {PARAMS['P001_self_check_max_wind_mps']}m/s")
        if rainfall_mm_h is None:
            ok = False
            messages.append("P002 自检雨量缺失，无法通过自检")
        elif rainfall_mm_h > PARAMS["P002_self_check_max_rainfall_mm_h"]:
            ok = False
            messages.append(
                f"P002 自检雨量{round(rainfall_mm_h, 2)}mm/h > "
                f"{PARAMS['P002_self_check_max_rainfall_mm_h']}mm/h"
            )
        else:
            messages.append(
                f"P002 自检雨量{round(rainfall_mm_h, 2)}mm/h <= "
                f"{PARAMS['P002_self_check_max_rainfall_mm_h']}mm/h"
            )
    messages.append("P003 能见度：JSON未提供字段，记录为待平台补充")
    if parse_temperature_range(temp):
        messages.append(f"P004 环境气温：{temp}，{PARAMS['P004_temperature_note']}")
    if humidity is not None:
        messages.append(f"湿度记录：{humidity}%")
    if ok and not messages:
        messages.append(f"天气可飞：{weather} / {wind}")
    elif ok:
        messages.insert(0, f"天气可飞：{weather} / {wind}")
    return ok, messages


def airport_ready(airport: Airport, date_str: Optional[str], work_order: Dict[str, Any]) -> Tuple[bool, List[str]]:
    reasons = []
    level = int(work_order.get("woker_order_level", 4) or 4)
    hard_block = False
    if airport.status != PARAMS["P008_ready_airport_status"]:
        reasons.append(f"P008 机场状态={airport.status}，非正常可飞")
        hard_block = True
    if airport.need_check != 0:
        reasons.append("机场开启自检，已按自检气象阈值校验")
    drone_ready = False
    if airport.drone.status == PARAMS["P008_ready_drone_status"]:
        drone_ready = True
        reasons.append(f"无人机状态={airport.drone.status}，待机可用")
    else:
        charge_remaining = airport.drone.charge_remaining_min
        if charge_remaining is not None and charge_remaining <= PARAMS["P018_charging_available_max_min"]:
            drone_ready = True
            reasons.append(
                f"P018 无人机充电中，剩余恢复{charge_remaining}min，"
                f"<= {PARAMS['P018_charging_available_max_min']}min，按可用处理"
            )
        else:
            reasons.append(f"P008/P018 无人机状态={airport.drone.status}，非待机且不可快速恢复")
            hard_block = True
    threshold = min_takeoff_battery(work_order)
    if airport.drone.battery_pct < threshold:
        reasons.append(f"P006 起飞电量{airport.drone.battery_pct}% < {threshold}%")
        hard_block = True
    if airport.current_task_id:
        incoming_level = PARAMS["P013_preempt_incoming_level"]
        current_priority = airport.task_priority
        can_preempt = (
            level == incoming_level
            and current_priority is not None
            and current_priority > level
            and airport.task_progress < PARAMS["P013_preempt_progress_guard_pct"]
        )
        if can_preempt:
            reasons.append(
                f"P013 1级工单可抢占低优先级当前任务{airport.current_task_id}"
                f"（当前优先级{current_priority}，进度{airport.task_progress}%）"
            )
        else:
            reasons.append(
                f"P013 当前任务{airport.current_task_id}不可抢占："
                f"当前优先级{current_priority if current_priority is not None else '缺失'}，"
                f"新工单优先级{level}，进度{airport.task_progress}%"
            )
            hard_block = True
    if airport.cross_railway:
        reasons.append("跨铁路机场：纳入风险提示，核心demo不做硬否决")
    ok_w, weather_msgs = weather_ok(airport, date_str, work_order)
    if not ok_w:
        reasons.extend(weather_msgs)
        hard_block = True
    elif airport.need_check:
        reasons.extend(weather_msgs)
    return (not hard_block) and ok_w and drone_ready, reasons or weather_msgs


def nearest_neighbor_route(airport: Airport, points: List[Any]) -> List[Any]:
    remaining = points[:]
    ordered: List[TargetPoint] = []
    cur_lon, cur_lat = airport.lon, airport.lat
    while remaining:
        nxt = min(remaining, key=lambda p: haversine_m(cur_lon, cur_lat, p.lon, p.lat))
        ordered.append(nxt)
        remaining.remove(nxt)
        cur_lon, cur_lat = nxt.lon, nxt.lat
    return ordered


def improve_route_2opt(airport: Airport, ordered: List[Any]) -> List[Any]:
    """在最近邻初始解上做轻量 2-opt，保持机场往返和任务对象完整。"""
    if len(ordered) < 3:
        return ordered[:]
    best = ordered[:]
    best_distance = route_distance_m(airport, best)
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                candidate = best[:i] + list(reversed(best[i:j + 1])) + best[j + 1:]
                distance = route_distance_m(airport, candidate)
                if distance + 0.1 < best_distance:
                    best, best_distance = candidate, distance
                    improved = True
                    break
            if improved:
                break
    return best


def _candidate_rows_for_point(
    point: TargetPoint,
    airports: List[Airport],
    date_str: Optional[str],
    work_order: Dict[str, Any],
) -> Tuple[List[Airport], List[Dict[str, Any]]]:
    rows = []
    feasible_airports = []
    for airport in airports:
        ready, reasons = airport_ready(airport, date_str, work_order)
        in_radius = all(
            _route_point_distance_to_airport(airport, raw) <= airport.radius_m
            for raw in point.route_points
        )
        feasible = ready and in_radius
        rows.append({
            "airport_uid": airport.uid,
            "airport_name": airport.name,
            "in_coverage": in_radius,
            "resource_ready": ready,
            "compliant": feasible,
            "feasible": feasible,
            "reason": "可覆盖且资源可用" if feasible else "; ".join(reasons) + ("" if in_radius else "; 无法覆盖全部航点"),
        })
        if feasible:
            feasible_airports.append(airport)
    return feasible_airports, rows


def optimize_airport_assignment(
    points: List[TargetPoint],
    airports: List[Airport],
    date_str: Optional[str],
    work_order: Dict[str, Any],
    scheme_name: str,
) -> Tuple[Dict[str, Airport], Dict[str, List[Dict[str, Any]]]]:
    """全局机场分配：小规模用分支定界，大规模回退为带负载项的贪心。

    这是一个无外部依赖的整数分配模型。规则先过滤不可行机场，优化器只在
    合规候选集合中最小化距离、耗时、电量风险和机场负载不均衡。
    """
    candidate_map: Dict[str, List[Airport]] = {}
    rows_map: Dict[str, List[Dict[str, Any]]] = {}
    for point in points:
        candidates, rows = _candidate_rows_for_point(point, airports, date_str, work_order)
        candidate_map[point.group_id] = candidates
        rows_map[point.group_id] = rows

    feasible_points = [p for p in points if candidate_map.get(p.group_id)]
    assignment: Dict[str, Airport] = {}
    loads = {a.uid: 0 for a in airports}

    def cost(point: TargetPoint, airport: Airport, load: int) -> float:
        distance = route_distance_m(airport, [point]) / 1000.0
        _, flight_min, work_min, duration = estimate_sortie(airport, [point])
        battery_use = (flight_min + work_min) / max(airport.drone.battery_life_min, 1.0)
        if scheme_name == "energy_first":
            return distance + duration * 0.05 + battery_use * 30 + load * 0.15
        if scheme_name == "time_first":
            return distance * 0.5 + duration + load * 0.1
        return distance + duration * 0.1 + battery_use * 10 + load * 0.25

    # 先分配候选最少的任务，有利于分支定界尽早发现不可行组合。
    ordered_points = sorted(feasible_points, key=lambda p: len(candidate_map[p.group_id]))
    exact_limit = int(PARAMS["P_ASSIGNMENT_EXACT_LIMIT"])
    if len(ordered_points) <= exact_limit:
        best_score = float("inf")
        best_assignment: Dict[str, Airport] = {}
        node_count = 0
        stopped = False

        def search(index: int, score: float) -> None:
            nonlocal best_score, best_assignment, node_count, stopped
            if stopped:
                return
            node_count += 1
            if node_count > int(PARAMS["P_ASSIGNMENT_NODE_LIMIT"]):
                stopped = True
                return
            if score >= best_score:
                return
            if index == len(ordered_points):
                best_score = score
                best_assignment = assignment.copy()
                return
            point = ordered_points[index]
            for airport in sorted(
                candidate_map[point.group_id],
                key=lambda a: cost(point, a, loads[a.uid]),
            ):
                assignment[point.group_id] = airport
                loads[airport.uid] += len(point.route_points)
                search(index + 1, score + cost(point, airport, loads[airport.uid] - len(point.route_points)))
                loads[airport.uid] -= len(point.route_points)
                assignment.pop(point.group_id, None)

        search(0, 0.0)
        assignment = best_assignment
        if not assignment:
            for point in ordered_points:
                airport = min(
                    candidate_map[point.group_id],
                    key=lambda a: cost(point, a, loads[a.uid]),
                )
                assignment[point.group_id] = airport
                loads[airport.uid] += len(point.route_points)
    else:
        for point in ordered_points:
            airport = min(
                candidate_map[point.group_id],
                key=lambda a: cost(point, a, loads[a.uid]),
            )
            assignment[point.group_id] = airport
            loads[airport.uid] += len(point.route_points)

    return assignment, rows_map


def route_distance_m(airport: Airport, ordered: List[Any]) -> float:
    total = 0.0
    cur_lon, cur_lat = airport.lon, airport.lat
    for obj in ordered:
        for raw in obj.route_points:
            lon = float(raw.get("lon"))
            lat = float(raw.get("lat"))
            total += haversine_m(cur_lon, cur_lat, lon, lat)
            cur_lon, cur_lat = lon, lat
    total += haversine_m(cur_lon, cur_lat, airport.lon, airport.lat)
    return total


def estimate_sortie(airport: Airport, ordered: List[Any]) -> Tuple[float, float, float, float]:
    distance_m = route_distance_m(airport, ordered)
    flight_min = distance_m / airport.drone.speed_mps / 60
    work_min = sum(work_minutes_for_item(item) for item in ordered)
    safety_min = max(flight_min * PARAMS["P010_safety_ratio"], PARAMS["P011_safety_fixed_min"])
    total_min = PARAMS["P040_prepare_min"] + flight_min + work_min + safety_min
    return distance_m, flight_min, work_min, total_min if ordered else 0.0


def work_minutes_for_item(item: Any) -> float:
    total_points = getattr(item, "full_waypoint_count", 0) or len(getattr(item, "route_points", []) or [])
    point_count = len(getattr(item, "route_points", []) or [])
    if point_count <= 0:
        return 0.0
    return PARAMS["P039_work_sec_per_waypoint"] * point_count / 60


def split_into_sorties(airport: Airport, ordered: List[Any], max_use_ratio: float) -> List[List[Any]]:
    max_min = airport.drone.battery_life_min * max_use_ratio
    sorties: List[List[TargetPoint]] = []
    current: List[TargetPoint] = []
    for point in ordered:
        trial = current + [point]
        _, _, _, trial_min = estimate_sortie(airport, trial)
        if current and trial_min > max_min:
            sorties.append(current)
            current = [point]
        else:
            current = trial
    if current:
        sorties.append(current)
    return sorties


def optimize_sortie_partition(
    airport: Airport,
    ordered: List[Any],
    max_use_ratio: float,
) -> List[List[Any]]:
    """固定任务顺序后，用动态规划寻找最少的连续架次分段。"""
    if len(ordered) < 2:
        return [ordered[:]] if ordered else []
    max_min = airport.drone.battery_life_min * max_use_ratio
    count = len(ordered)
    best_count = [float("inf")] * (count + 1)
    previous = [-1] * (count + 1)
    best_count[0] = 0
    for end in range(1, count + 1):
        for start in range(end - 1, -1, -1):
            segment = ordered[start:end]
            _, _, _, total_min = estimate_sortie(airport, segment)
            if total_min > max_min:
                continue
            candidate_count = best_count[start] + 1
            if candidate_count < best_count[end]:
                best_count[end] = candidate_count
                previous[end] = start
    if previous[count] < 0:
        return split_into_sorties(airport, ordered, max_use_ratio)
    result: List[List[Any]] = []
    cursor = count
    while cursor > 0:
        start = previous[cursor]
        result.append(ordered[start:cursor])
        cursor = start
    result.reverse()
    return result


def split_item_by_waypoints_for_relay(item: Any, relay_legs: int, airport_uid: str) -> List[WorkSegment]:
    route_points = list(getattr(item, "route_points", []) or [])
    total = len(route_points)
    if relay_legs <= 1 or total <= 1:
        if isinstance(item, WorkSegment):
            item.segment_index = 1
            item.segment_count = 1
            item.full_waypoint_count = item.full_waypoint_count or total
            return [item]
        return [
            WorkSegment(
                item.group_id,
                item.group_name,
                airport_uid,
                route_points,
                full_waypoint_count=total,
                segment_index=1,
                segment_count=1,
            )
        ]
    chunks = []
    chunk_size = math.ceil(total / relay_legs)
    for idx in range(relay_legs):
        start = idx * chunk_size
        end = min(start + chunk_size, total)
        chunk = route_points[start:end]
        if not chunk:
            continue
        chunks.append(
            WorkSegment(
                item.group_id,
                item.group_name,
                airport_uid,
                chunk,
                full_waypoint_count=total,
                segment_index=idx + 1,
                segment_count=relay_legs,
            )
        )
    return chunks


def optimize_relay_segments(
    item: Any,
    airport: Airport,
    max_use_ratio: float,
) -> List[WorkSegment]:
    """按真实往返航程寻找满足续航硬约束的最少连续航点分段。"""
    route_points = list(getattr(item, "route_points", []) or [])
    total = len(route_points)
    if not route_points:
        return []
    max_min = airport.drone.battery_life_min * max_use_ratio
    best: List[Optional[Tuple[int, float]]] = [None] * (total + 1)
    previous = [-1] * (total + 1)
    best[0] = (0, 0.0)
    for end in range(1, total + 1):
        for start in range(end - 1, -1, -1):
            if best[start] is None:
                continue
            segment = WorkSegment(
                item.group_id,
                item.group_name,
                airport.uid,
                route_points[start:end],
                full_waypoint_count=total,
            )
            _, _, _, duration = estimate_sortie(airport, [segment])
            use_pct = duration / max(airport.drone.battery_life_min, 1.0) * 100
            remaining_pct = 100 - use_pct
            if (
                duration > max_min
                or use_pct > airport.drone.battery_pct
                or remaining_pct <= PARAMS["battery_return_pct"]
            ):
                continue
            candidate = (best[start][0] + 1, best[start][1] + duration)
            if best[end] is None or candidate < best[end]:
                best[end] = candidate
                previous[end] = start
    if best[total] is None:
        return [WorkSegment(
            item.group_id,
            item.group_name,
            airport.uid,
            route_points,
            full_waypoint_count=total,
        )]
    ranges = []
    cursor = total
    while cursor > 0:
        start = previous[cursor]
        ranges.append((start, cursor))
        cursor = start
    ranges.reverse()
    segment_count = len(ranges)
    return [
        WorkSegment(
            item.group_id,
            item.group_name,
            airport.uid,
            route_points[start:end],
            full_waypoint_count=total,
            segment_index=index,
            segment_count=segment_count,
        )
        for index, (start, end) in enumerate(ranges, start=1)
    ]


def schedule_plans(
    plans: List[Dict[str, Any]],
    airports: List[Airport],
    scheme_name: str,
    window_start: Optional[datetime],
    window_end: Optional[datetime],
) -> Tuple[float, float]:
    """按资源串行排程；接力段保持顺序，独立架次按方案目标排序。"""
    airport_map = {a.uid: a for a in airports}
    groups: Dict[Tuple[str, str], Dict[int, List[Dict[str, Any]]]] = {}
    for plan in plans:
        resource = (plan["airport_uid"], plan["drone_id"])
        groups.setdefault(resource, {}).setdefault(plan["sortie_index"], []).append(plan)

    resource_finishes = []
    completion_values = []
    for resource, sortie_map in groups.items():
        blocks = []
        for sortie_index, block in sortie_map.items():
            block.sort(key=lambda p: p["relay_leg"])
            duration = sum(p["total_min"] + p["charging_duration_min"] for p in block)
            risk = max(p["battery_use_pct"] for p in block)
            blocks.append((sortie_index, block, duration, risk))
        schedule_optimized = PARAMS.get("P_ENABLE_SCHEDULE_OPTIMIZER", True)
        if schedule_optimized and scheme_name == "time_first":
            blocks.sort(key=lambda row: (row[2], row[0]))
        elif schedule_optimized and scheme_name == "energy_first":
            blocks.sort(key=lambda row: (row[3], row[2], row[0]))
        else:
            blocks.sort(key=lambda row: row[0])

        airport = airport_map[resource[0]]
        initial_wait = 0.0
        if airport.drone.status != PARAMS["P008_ready_drone_status"]:
            initial_wait = max(0.0, float(airport.drone.charge_remaining_min or 0.0))
        clock = max(initial_wait, float(PARAMS["P012_response_min"]))
        previous_finish: Optional[float] = None
        schedule_order = 0
        for _, block, _, _ in blocks:
            for plan in block:
                schedule_order += 1
                start = clock
                finish = start + plan["total_min"]
                plan["schedule_order"] = schedule_order
                plan["planned_takeoff_offset_min"] = round(start, 1)
                plan["planned_finish_offset_min"] = round(finish, 1)
                plan["charge_wait_before_takeoff_min"] = round(
                    0.0 if previous_finish is None else start - previous_finish, 1
                )
                planned_start = window_start + timedelta(minutes=start) if window_start else None
                planned_finish = window_start + timedelta(minutes=finish) if window_start else None
                recovery_finish = finish + plan["charging_duration_min"]
                plan["planned_start_time"] = fmt_dt(planned_start)
                plan["planned_end_time"] = fmt_dt(planned_finish)
                plan["resource_recovery_end_offset_min"] = round(recovery_finish, 1)
                plan["resource_recovery_end_time"] = fmt_dt(
                    window_start + timedelta(minutes=recovery_finish) if window_start else None
                )
                plan["within_work_order_window"] = bool(
                    planned_start
                    and planned_finish
                    and (window_end is None or planned_finish <= window_end)
                )
                completion_values.append(finish)
                previous_finish = finish
                clock = recovery_finish
        resource_finishes.append(previous_finish or 0.0)
    makespan = max(resource_finishes, default=0.0)
    average_completion = sum(completion_values) / len(completion_values) if completion_values else 0.0
    return round(makespan, 1), round(average_completion, 1)


def choose_airport(
    point: TargetPoint,
    airports: Iterable[Airport],
    date_str: Optional[str],
    work_order: Dict[str, Any],
    scheme_name: str,
    load: Dict[str, int],
) -> Tuple[Optional[Airport], List[Dict[str, Any]]]:
    rows = []
    ranked: List[Tuple[Tuple[float, ...], Airport]] = []
    for airport in airports:
        d = haversine_m(airport.lon, airport.lat, point.lon, point.lat)
        ready, reasons = airport_ready(airport, date_str, work_order)
        in_radius = d <= airport.radius_m
        feasible = ready and in_radius
        single_distance, single_flight, single_work, _ = estimate_sortie(airport, [point])
        single_use = (single_flight + single_work) / airport.drone.battery_life_min * 100
        rows.append(
            {
                "airport_uid": airport.uid,
                "airport_name": airport.name,
                "distance_m": round(d, 1),
                "single_object_route_m": round(single_distance, 1),
                "single_object_battery_use_pct": round(single_use, 1),
                "inspection_radius_m": airport.radius_m,
                "in_coverage": in_radius,
                "resource_ready": ready,
                "compliant": feasible,
                "feasible": feasible,
                "reason": "可覆盖且资源可用" if feasible else "; ".join(reasons if ready else reasons) + ("" if in_radius else "; 超出覆盖范围"),
            }
        )
        if not feasible:
            continue
        if scheme_name == "balanced":
            key = (load.get(airport.uid, 0), d)
        elif scheme_name == "energy_first":
            key = (single_use, single_distance)
        else:
            key = (d,)
        ranked.append((key, airport))
    ranked.sort(key=lambda x: x[0])
    return (ranked[0][1] if ranked else None), rows


def _route_point_distance_to_airport(airport: Airport, raw: Dict[str, Any]) -> float:
    return haversine_m(airport.lon, airport.lat, float(raw.get("lon")), float(raw.get("lat")))


def split_object_by_airport(
    point: TargetPoint,
    airports: List[Airport],
    date_str: Optional[str],
    work_order: Dict[str, Any],
    scheme_name: str,
    load: Dict[str, int],
) -> Tuple[List[WorkSegment], List[Dict[str, Any]]]:
    rows = []
    ready_airports = []
    for airport in airports:
        ready, reasons = airport_ready(airport, date_str, work_order)
        in_radius = any(_route_point_distance_to_airport(airport, raw) <= airport.radius_m for raw in point.route_points)
        feasible = ready and in_radius
        rows.append(
            {
                "airport_uid": airport.uid,
                "airport_name": airport.name,
                "in_coverage": in_radius,
                "resource_ready": ready,
                "compliant": feasible,
                "feasible": feasible,
                "reason": "可覆盖对象部分航点" if feasible else "; ".join(reasons) + ("" if in_radius else "; 无覆盖航点"),
            }
        )
        if feasible:
            ready_airports.append(airport)
    if not ready_airports:
        return [], rows

    buckets: Dict[str, List[Dict[str, Any]]] = {}
    for raw in point.route_points:
        feasible = [
            a for a in ready_airports
            if _route_point_distance_to_airport(a, raw) <= a.radius_m
        ]
        if not feasible:
            return [], rows
        if scheme_name == "balanced":
            chosen = min(feasible, key=lambda a: (load.get(a.uid, 0) + len(buckets.get(a.uid, [])), _route_point_distance_to_airport(a, raw)))
        elif scheme_name == "energy_first":
            chosen = min(feasible, key=lambda a: (_route_point_distance_to_airport(a, raw) / a.drone.battery_life_min, _route_point_distance_to_airport(a, raw)))
        else:
            chosen = min(feasible, key=lambda a: _route_point_distance_to_airport(a, raw))
        buckets.setdefault(chosen.uid, []).append(raw)

    segments = [
        WorkSegment(
            point.group_id,
            point.group_name,
            airport_uid,
            route_points,
            full_waypoint_count=len(point.route_points),
        )
        for airport_uid, route_points in buckets.items()
    ]
    return segments, rows


def _battery_status(remaining_pct: float) -> Dict[str, Any]:
    remain = round(remaining_pct, 1)
    if remain <= PARAMS["battery_return_pct"]:
        level = "返航"
    elif remain <= PARAMS["battery_relay_pct"]:
        level = "接力"
    elif remain <= PARAMS["battery_warn_pct"]:
        level = "预警"
    else:
        level = "正常"
    return {"remaining_pct": remain, "level": level}


def build_scheme(data: Dict[str, Any], scheme_name: str) -> Dict[str, Any]:
    scheme_info = SCHEME_INFO[scheme_name]
    work_order = data.get("work_order", {})
    date_str = str(work_order.get("start_date", ""))[:10] or None
    window_start = parse_dt(work_order.get("start_date"))
    window_end = parse_dt(work_order.get("end_date"))
    airports = parse_airports(data)
    targets = parse_targets(data)
    assignments: Dict[str, List[Any]] = {}
    target_rows = []
    rejected = []
    decision_rows = []
    elapsed_min = 0.0

    optimized_assignment: Dict[str, Airport] = {}
    optimized_candidates: Dict[str, List[Dict[str, Any]]] = {}
    if PARAMS.get("P_ENABLE_ASSIGNMENT_OPTIMIZER", True):
        optimized_assignment, optimized_candidates = optimize_airport_assignment(
            targets, airports, date_str, work_order, scheme_name
        )

    for point in targets:
        load = {k: sum(len(getattr(x, "route_points", [])) for x in v) for k, v in assignments.items()}
        if scheme_name == "balanced" and optimized_assignment:
            airport = optimized_assignment.get(point.group_id)
            candidates = optimized_candidates.get(point.group_id, [])
            candidate_airports = [c for c in candidates if c.get("in_coverage")]
            compliant_airports = [c for c in candidates if c.get("compliant")]
            if airport is None:
                rejected.append({
                    "target": point.name,
                    "lon": point.lon,
                    "lat": point.lat,
                    "candidates": candidates,
                    "candidate_airports": candidate_airports,
                    "compliant_airports": compliant_airports,
                    "reason": "无合规机场可覆盖该对象",
                })
                continue
            assignments.setdefault(airport.uid, []).append(point)
            route_id = point.group_id
            distance_m, flight_min, work_min, total_min = estimate_sortie(airport, [point])
            battery_use_pct = round(
                (flight_min + work_min) / airport.drone.battery_life_min * 100,
                1,
            )
            start_offset_min = round(elapsed_min, 1)
            elapsed_min += total_min
            target_rows.append({
                "target": point.name,
                "group": point.group_name,
                "assigned_airport_uid": airport.uid,
                "assigned_airport_name": airport.name,
                "drone_id": airport.drone.drone_id,
                "route_id": route_id,
                "waypoint_count": len(point.route_points),
                "distance_to_airport_m": round(haversine_m(airport.lon, airport.lat, point.lon, point.lat), 1),
                "distance_m": round(distance_m, 1),
                "flight_min": round(flight_min, 1),
                "work_min": round(work_min, 1),
                "total_min": round(total_min, 1),
                "battery_use_pct": battery_use_pct,
                "start_offset_min": start_offset_min,
                "end_offset_min": round(elapsed_min, 1),
            })
            decision_rows.append({
                "target": point.name,
                "route_id": route_id,
                "airport_uid": airport.uid,
                "airport_name": airport.name,
                "drone_id": airport.drone.drone_id,
                "candidate_airports": candidate_airports,
                "compliant_airports": compliant_airports,
                "planned_takeoff_min": start_offset_min,
                "planned_finish_min": round(elapsed_min, 1),
                "flight_min": round(flight_min, 1),
                "work_min": round(work_min, 1),
                "total_min": round(total_min, 1),
                "reason": "全局机场分配优化：距离 + 时长 + 电量风险 + 负载均衡",
            })
        elif scheme_name in ("time_first", "energy_first"):
            segments, candidates = split_object_by_airport(point, airports, date_str, work_order, scheme_name, load)
            candidate_airports = [c for c in candidates if c.get("in_coverage")]
            compliant_airports = [c for c in candidates if c.get("compliant")]
            if not segments:
                rejected.append({
                    "target": point.name,
                    "lon": point.lon,
                    "lat": point.lat,
                    "candidates": candidates,
                    "candidate_airports": candidate_airports,
                    "compliant_airports": compliant_airports,
                    "reason": "无合规机场可覆盖该对象全部航点",
                })
                continue
            for segment in segments:
                airport = next(a for a in airports if a.uid == segment.airport_uid)
                assignments.setdefault(airport.uid, []).append(segment)
                route_id = point.group_id
                single_distance = route_distance_m(airport, [segment])
                _, flight_min, work_min, total_min = estimate_sortie(airport, [segment])
                battery_use_pct = round(
                    (flight_min + work_min) / airport.drone.battery_life_min * 100,
                    1,
                )
                start_offset_min = round(elapsed_min, 1)
                elapsed_min += total_min
                target_rows.append(
                    {
                        "target": point.name,
                        "group": point.group_name,
                        "assigned_airport_uid": airport.uid,
                        "assigned_airport_name": airport.name,
                        "drone_id": airport.drone.drone_id,
                        "route_id": route_id,
                        "waypoint_count": len(segment.route_points),
                        "distance_to_airport_m": round(haversine_m(airport.lon, airport.lat, point.lon, point.lat), 1),
                        "distance_m": round(single_distance, 1),
                        "flight_min": round(flight_min, 1),
                        "work_min": round(work_min, 1),
                        "total_min": round(total_min, 1),
                        "battery_use_pct": battery_use_pct,
                        "start_offset_min": start_offset_min,
                        "end_offset_min": round(elapsed_min, 1),
                    }
                )
                decision_rows.append({
                    "target": point.name,
                    "route_id": route_id,
                    "airport_uid": airport.uid,
                    "airport_name": airport.name,
                    "drone_id": airport.drone.drone_id,
                    "candidate_airports": candidate_airports,
                    "compliant_airports": compliant_airports,
                    "planned_takeoff_min": start_offset_min,
                    "planned_finish_min": round(elapsed_min, 1),
                    "flight_min": round(flight_min, 1),
                    "work_min": round(work_min, 1),
                    "total_min": round(total_min, 1),
                    "reason": "就近 + 覆盖 + 续航校验后指派",
                })
        else:
            airport, candidates = choose_airport(point, airports, date_str, work_order, scheme_name, load)
            candidate_airports = [c for c in candidates if c.get("in_coverage")]
            compliant_airports = [c for c in candidates if c.get("compliant")]
            if airport is None:
                rejected.append({
                    "target": point.name,
                    "lon": point.lon,
                    "lat": point.lat,
                    "candidates": candidates,
                    "candidate_airports": candidate_airports,
                    "compliant_airports": compliant_airports,
                    "reason": "无合规机场可覆盖该对象",
                })
                continue
            assignments.setdefault(airport.uid, []).append(point)
            route_id = point.group_id
            distance_m, flight_min, work_min, total_min = estimate_sortie(airport, [point])
            battery_use_pct = round(
                (flight_min + work_min) / airport.drone.battery_life_min * 100,
                1,
            )
            start_offset_min = round(elapsed_min, 1)
            elapsed_min += total_min
            target_rows.append(
                {
                    "target": point.name,
                    "group": point.group_name,
                    "assigned_airport_uid": airport.uid,
                    "assigned_airport_name": airport.name,
                    "drone_id": airport.drone.drone_id,
                    "route_id": route_id,
                    "waypoint_count": len(point.route_points),
                    "distance_to_airport_m": round(haversine_m(airport.lon, airport.lat, point.lon, point.lat), 1),
                    "distance_m": round(distance_m, 1),
                    "flight_min": round(flight_min, 1),
                    "work_min": round(work_min, 1),
                    "total_min": round(total_min, 1),
                    "battery_use_pct": battery_use_pct,
                    "start_offset_min": start_offset_min,
                    "end_offset_min": round(elapsed_min, 1),
                }
            )
            decision_rows.append({
                "target": point.name,
                "route_id": route_id,
                "airport_uid": airport.uid,
                "airport_name": airport.name,
                "drone_id": airport.drone.drone_id,
                "candidate_airports": candidate_airports,
                "compliant_airports": compliant_airports,
                "planned_takeoff_min": start_offset_min,
                "planned_finish_min": round(elapsed_min, 1),
                "flight_min": round(flight_min, 1),
                "work_min": round(work_min, 1),
                "total_min": round(total_min, 1),
                "reason": "就近 + 覆盖 + 续航校验后指派",
            })

    plans = []
    airport_map = {a.uid: a for a in airports}
    for airport_uid, pts in assignments.items():
        airport = airport_map[airport_uid]
        ordered = improve_route_2opt(airport, nearest_neighbor_route(airport, pts))
        max_use_ratio = scheme_max_use_ratio(scheme_name)
        if PARAMS.get("P_ENABLE_SORTIE_OPTIMIZER", True):
            sorties = optimize_sortie_partition(airport, ordered, max_use_ratio)
        else:
            sorties = split_into_sorties(airport, ordered, max_use_ratio)
        for idx, sortie in enumerate(sorties, start=1):
            distance_m, flight_min, work_min, total_min = estimate_sortie(airport, sortie)
            max_min = airport.drone.battery_life_min * max_use_ratio
            relay_reason = ""
            if total_min > max_min and len(sortie) == 1:
                if PARAMS.get("P_ENABLE_SORTIE_OPTIMIZER", True):
                    relay_segments = optimize_relay_segments(sortie[0], airport, max_use_ratio)
                else:
                    safety_min = max(
                        flight_min * PARAMS["P010_safety_ratio"],
                        PARAMS["P011_safety_fixed_min"],
                    )
                    max_work_per_leg = max_min - flight_min - safety_min - PARAMS["P040_prepare_min"]
                    relay_count = math.ceil(work_min / max_work_per_leg) if max_work_per_leg > 0 else 1
                    relay_segments = split_item_by_waypoints_for_relay(
                        sortie[0], relay_count, airport.uid
                    )
                relay_legs = len(relay_segments)
                execution_legs: List[List[Any]] = [[segment] for segment in relay_segments]
                relay_reason = (
                    f"完整执行约{round(total_min, 1)}min，超过本方案单段安全上限"
                    f"{round(max_min, 1)}min；按真实往返航程优化为{relay_legs}段，每段返航后再接续。"
                )
            else:
                execution_legs = [sortie]
                relay_legs = 1
            for seg_index, leg_items in enumerate(execution_legs, start=1):
                seg_distance_m, seg_flight_min, seg_work_min, _ = estimate_sortie(airport, leg_items)
                safety_min = max(seg_flight_min * PARAMS["P010_safety_ratio"], PARAMS["P011_safety_fixed_min"])
                leg_total_min = PARAMS["P040_prepare_min"] + seg_flight_min + seg_work_min + safety_min
                battery_use_pct = round(
                    (seg_flight_min + seg_work_min) / airport.drone.battery_life_min * 100,
                    1,
                )
                required_battery_pct = (
                    (seg_flight_min + seg_work_min + safety_min)
                    / airport.drone.battery_life_min
                    * 100
                )
                battery_meta = _battery_status(
                    airport.drone.battery_pct - battery_use_pct
                )
                feasible = (
                    required_battery_pct <= airport.drone.battery_pct
                    and leg_total_min <= max_min
                    and battery_meta["remaining_pct"] > PARAMS["battery_return_pct"]
                )
                start_wp = leg_items[0].route_points[0].get("name") if leg_items[0].route_points else ""
                end_wp = leg_items[-1].route_points[-1].get("name") if leg_items[-1].route_points else ""
                segment_index = getattr(leg_items[0], "segment_index", 1) if len(leg_items) == 1 else 1
                segment_count = getattr(leg_items[0], "segment_count", 1) if len(leg_items) == 1 else 1
                plans.append(
                    {
                        "scheme_name": scheme_name,
                        "airport_uid": airport.uid,
                        "airport_name": airport.name,
                        "airport_lon": airport.lon,
                        "airport_lat": airport.lat,
                        "drone_id": airport.drone.drone_id,
                        "drone_name": airport.drone.name,
                        "sortie_index": idx,
                        "relay_leg": seg_index,
                        "relay_legs": relay_legs,
                        "relay_reason": relay_reason,
                        "point_count": sum(len(item.route_points) for item in leg_items),
                        "object_names": [item.group_name for item in leg_items],
                        "route": [
                            {
                                "obj_id": item.group_id,
                                "obj_name": item.group_name,
                                "center_lon": item.lon,
                                "center_lat": item.lat,
                                "waypoint_count": len(item.route_points),
                                "first_waypoint": item.route_points[0].get("name") if item.route_points else "",
                                "last_waypoint": item.route_points[-1].get("name") if item.route_points else "",
                                "waypoints": item.route_points,
                            }
                            for item in leg_items
                        ],
                        "distance_m": round(seg_distance_m, 1),
                        "flight_min": round(seg_flight_min, 1),
                        "work_min": round(seg_work_min, 1),
                        "prepare_min": PARAMS["P040_prepare_min"],
                        "safety_min": round(safety_min, 1),
                        "full_task_min": round(seg_flight_min + seg_work_min + safety_min + PARAMS["P040_prepare_min"], 1),
                        "max_allowed_min": round(max_min, 1),
                        "total_min": round(leg_total_min, 1),
                        "battery_use_pct": battery_use_pct,
                        "battery_remaining_pct": battery_meta["remaining_pct"],
                        "battery_level": battery_meta["level"],
                        "battery_available_pct": airport.drone.battery_pct,
                        "charging_duration_min": airport.drone.charging_duration_min,
                        "feasible_after_endurance_check": feasible,
                        "segment_index": segment_index,
                        "segment_count": segment_count,
                        "segment_start_waypoint": start_wp,
                        "segment_end_waypoint": end_wp,
                    }
                )

    status = "success"
    if rejected:
        status = "partial" if plans else "infeasible"
    if any(not p["feasible_after_endurance_check"] for p in plans):
        status = "partial"

    estimated_completion_min, average_completion_min = schedule_plans(
        plans, airports, scheme_name, window_start, window_end
    )
    estimated_completion_time = (
        fmt_dt(window_start + timedelta(minutes=estimated_completion_min))
        if window_start and estimated_completion_min
        else ""
    )
    within_window = True
    if window_end and estimated_completion_time:
        within_window = parse_dt(estimated_completion_time) <= window_end
    if any(not p.get("within_work_order_window", True) for p in plans):
        status = "partial"

    return {
        "scheme_name": scheme_name,
        "scheme_label": scheme_info["label"],
        "description": scheme_info["description"],
        "status": status,
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "work_order": {
            "name": work_order.get("woker_order_name"),
            "guid": work_order.get("woker_order_guid"),
            "level": work_order.get("woker_order_level"),
            "execution": work_order.get("woker_order_execution"),
            "day_or_night": work_order.get("day_or_night"),
            "start_date": work_order.get("start_date"),
            "end_date": work_order.get("end_date"),
            "response_target_min": PARAMS["P012_response_min"],
            "target_count": len(targets),
        },
        "summary": {
            "airport_count": len(airports),
            "assigned_target_count": len({r["target"] for r in target_rows}),
            "unassigned_target_count": len(rejected),
            "used_airport_count": len(assignments),
            "segment_count": len(target_rows),
            "sortie_count": len(plans),
            "total_route_distance_m": round(sum(p["distance_m"] for p in plans), 1),
            "max_plan_duration_min": round(max((p["total_min"] for p in plans), default=0), 1),
            "min_battery_remaining_pct": round(min((p["battery_remaining_pct"] for p in plans), default=0), 1),
            "avg_battery_remaining_pct": round(
                sum(p["battery_remaining_pct"] for p in plans) / len(plans), 1
            ) if plans else 0,
            "warn_or_worse_count": sum(1 for p in plans if p["battery_level"] in ("预警", "接力", "返航")),
            "return_risk_count": sum(1 for p in plans if p["battery_level"] == "返航"),
            "estimated_completion_min": estimated_completion_min,
            "average_sortie_completion_min": average_completion_min,
            "estimated_start_time": fmt_dt(window_start),
            "estimated_completion_time": estimated_completion_time,
            "work_order_end_time": fmt_dt(window_end),
            "within_work_order_window": within_window,
            "assignment_optimizer": bool(PARAMS.get("P_ENABLE_ASSIGNMENT_OPTIMIZER", True) and scheme_name == "balanced"),
            "sortie_optimizer": bool(PARAMS.get("P_ENABLE_SORTIE_OPTIMIZER", True)),
            "schedule_optimizer": bool(PARAMS.get("P_ENABLE_SCHEDULE_OPTIMIZER", True)),
        },
        "plans": sorted(plans, key=lambda p: p["airport_name"]),
        "assignments": target_rows,
        "decision_chain": decision_rows,
        "unassigned": rejected,
    }


def build_rule_audit(data: Dict[str, Any]) -> List[Dict[str, str]]:
    work_order = data.get("work_order", {})
    execution = int(work_order.get("woker_order_execution", 1) or 1)
    battery_rule = (
        f"周期工单起飞电量>={PARAMS['P006_min_battery_periodic_pct']}%"
        if execution == 3
        else f"临时/立即工单起飞电量>={PARAMS['P006_min_battery_temp_pct']}%"
    )
    return [
        {
            "param": "P001",
            "json_fields": "airport_need_check, airport.wind_speed, airport_weather.windSpeed",
            "usage": f"周期任务按风力等级硬校验；机场自检时风速<={PARAMS['P001_self_check_max_wind_mps']}m/s",
        },
        {
            "param": "P002",
            "json_fields": "airport_need_check, airport.rainfall, airport_weather.weather",
            "usage": f"周期任务中雨及以上禁飞，小雨预警；机场自检时雨量<={PARAMS['P002_self_check_max_rainfall_mm_h']}mm/h，airport.rainfall按mm/min换算",
        },
        {"param": "P003", "json_fields": "JSON缺能见度字段", "usage": f"记录缺字段；阈值为{PARAMS['P003_visibility_min_m']}m"},
        {"param": "P004", "json_fields": "airport_weather.externalTemperature", "usage": "记录温度；平台未确认阈值，当前不硬否决"},
        {"param": "P005", "json_fields": "airport_weather.weather", "usage": "出现雷/雷暴关键字直接禁飞"},
        {"param": "P006", "json_fields": "drone.drone_battery, work_order.woker_order_execution", "usage": battery_rule},
        {
            "param": "P007",
            "json_fields": "drone.battery_life, 航线距离, 作业时长, 安全冗余",
            "usage": (
                "单架次总时长不得超过无人机台账满电续航；"
                "当前台账为0.5h（30min），三套方案可按返航/接力/预警线采用更保守上限"
            ),
        },
        {"param": "P008", "json_fields": "airport_status, drone.drone_status", "usage": "机场正常且无人机待机才可调度"},
        {"param": "P009", "json_fields": "无日出勤统计字段", "usage": "规则表确认不限，当前不限制"},
        {"param": "P010/P011", "json_fields": "航线距离, drone.drone_vertical", "usage": f"安全冗余=max(飞行时长×比例, {PARAMS['P011_safety_fixed_min']}min)"},
        {
            "param": "P012",
            "json_fields": "work_order.woker_order_level, start_date",
            "usage": f"记录响应目标：接入后约{PARAMS['P012_response_min']}min开始准备，时间轴按该目标顺排",
        },
        {"param": "P013", "json_fields": "current_task_id/current_task, task_priority, task_progress, woker_order_level", "usage": "1级工单仅可抢占未到进度保护线的低优先级任务，不可抢占同为1级的任务"},
        {"param": "P014", "json_fields": "JSON缺任务类型白名单", "usage": "白名单为空，当前不额外限制抢占"},
        {"param": "P015", "json_fields": "woker_order_level, woker_order_execution", "usage": "1级偏自动推荐，2-3级可人工确认，4级偏排班"},
        {"param": "P028", "json_fields": "woder_order_detail[].obj_data[].lon/lat", "usage": "线路航点跨度用于协同拆分评估"},
        {"param": "P029", "json_fields": "woder_order_detail", "usage": f"对象数>={PARAMS['P029_multi_object_trigger']}触发多机场/多机协同评估"},
        {"param": "P037", "json_fields": "drone.battery_life", "usage": "满电续航是所有续航和接力计算基准"},
        {"param": "P038", "json_fields": "drone.drone_vertical, obj_data速度缺省", "usage": "去首航点和返航按JSON速度，航点内速度字段缺省时沿用该速度"},
        {"param": "P039", "json_fields": "woder_order_detail[].obj_data", "usage": f"作业时长=航点数×{PARAMS['P039_work_sec_per_waypoint']}秒，按实际分段航点数计算"},
        {"param": "P040", "json_fields": "无显式字段", "usage": f"每段加入起飞前准备{PARAMS['P040_prepare_min']}min"},
        {"param": "P041", "json_fields": "inspection_radius", "usage": "覆盖预筛使用机场台账半径"},
        {"param": "P042", "json_fields": "obj_data[].lon/lat", "usage": "规则表暂不考虑，当前不做顺路合并"},
        {"param": "P043", "json_fields": "drone.charging_duration", "usage": "同一无人机连续架次之间加入满充/换电恢复等待时间"},
        {"param": "扩展", "json_fields": "airport_cross_railway, day_or_night, humidity, altitude", "usage": "跨铁路、湿度、高度进入风险/展示；无硬阈值时不否决"},
    ]


def recommendation_key(scheme: Dict[str, Any], work_order: Dict[str, Any]) -> Tuple[Any, ...]:
    s = scheme["summary"]
    level = int(work_order.get("woker_order_level", 4) or 4)
    execution = int(work_order.get("woker_order_execution", 1) or 1)
    if level == 1 or execution == 1:
        return (
            scheme["status"] != "success",
            s["return_risk_count"],
            s["sortie_count"],
            s["max_plan_duration_min"],
            s["warn_or_worse_count"],
            s["total_route_distance_m"],
        )
    return (
        scheme["status"] != "success",
        s["return_risk_count"],
        s["warn_or_worse_count"],
        -s["min_battery_remaining_pct"],
        s["sortie_count"],
        s["total_route_distance_m"],
    )


def build_recommendation_reason(recommended: Dict[str, Any], schemes: List[Dict[str, Any]], work_order: Dict[str, Any]) -> str:
    r = recommended["summary"]
    level = int(work_order.get("woker_order_level", 4) or 4)
    execution = int(work_order.get("woker_order_execution", 1) or 1)
    if len(schemes) == 1:
        return (
            f"仅{recommended['scheme_label']}满足全部覆盖、续航和返航安全校验；"
            f"其余策略在拆分后出现更高电量风险或返航风险，因此推荐该方案。"
        )
    basis = "该工单为1级/立即执行，推荐优先级为：无返航风险 > 架次少 > 最长耗时短 > 电量风险少 > 总距离短。"
    if not (level == 1 or execution == 1):
        basis = "该工单非最高时效优先，推荐优先级为：无返航风险 > 电量风险少 > 最小电量余量高 > 架次少 > 总距离短。"

    comparisons = []
    for other in schemes:
        if other["scheme_name"] == recommended["scheme_name"]:
            continue
        o = other["summary"]
        bits = []
        if r["sortie_count"] != o["sortie_count"]:
            bits.append(f"架次{r['sortie_count']}比{other['scheme_label']}的{o['sortie_count']}更少")
        if r["max_plan_duration_min"] != o["max_plan_duration_min"]:
            bits.append(f"最长耗时{r['max_plan_duration_min']}min vs {o['max_plan_duration_min']}min")
        if r["warn_or_worse_count"] != o["warn_or_worse_count"]:
            bits.append(f"预警/接力/返航段{r['warn_or_worse_count']} vs {o['warn_or_worse_count']}")
        if r["total_route_distance_m"] != o["total_route_distance_m"]:
            bits.append(f"总距离{r['total_route_distance_m']}m vs {o['total_route_distance_m']}m")
        if not bits:
            bits.append("关键指标持平")
        comparisons.append(f"相对{other['scheme_label']}：" + "，".join(bits))

    return (
        f"{basis}最终选择{recommended['scheme_label']}：状态={recommended['status']}，"
        f"架次={r['sortie_count']}，最长耗时={r['max_plan_duration_min']}min，"
        f"最低电量余量={r['min_battery_remaining_pct']}%，总距离={r['total_route_distance_m']}m。"
        + " ".join(comparisons)
    )


def scheme_signature(scheme: Dict[str, Any]) -> Tuple[Any, ...]:
    """A business-facing signature: same assignment/relay shape means same scheme."""
    grouped = []
    for plan in sorted(
        scheme["plans"],
        key=lambda p: (
            p["airport_uid"],
            p["sortie_index"],
            p["relay_leg"],
            tuple(r["obj_id"] for r in p["route"]),
        ),
    ):
        grouped.append(
            (
                plan["airport_uid"],
                plan["drone_id"],
                plan["sortie_index"],
                plan["relay_leg"],
                plan["relay_legs"],
                tuple(
                    (
                        r["obj_id"],
                        r["first_waypoint"],
                        r["last_waypoint"],
                        r["waypoint_count"],
                    )
                    for r in plan["route"]
                ),
                round(plan["distance_m"], 1),
                round(plan["total_min"], 1),
                round(plan["battery_remaining_pct"], 1),
            )
        )
    summary = scheme["summary"]
    return (
        summary["assigned_target_count"],
        summary["unassigned_target_count"],
        summary["segment_count"],
        summary["used_airport_count"],
        summary["sortie_count"],
        tuple(grouped),
    )


def generate_schemes(data: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    attempted = [
        build_scheme(data, "balanced"),
        build_scheme(data, "time_first"),
        build_scheme(data, "energy_first"),
    ]
    generated: List[Dict[str, Any]] = []
    seen = set()
    for scheme in attempted:
        if scheme["status"] != "success":
            scheme["reject_reason"] = "未完整覆盖全部对象，或存在续航/资源硬约束风险。"
            continue
        signature = scheme_signature(scheme)
        if signature in seen:
            scheme["reject_reason"] = "与已生成方案的机场分配、接力段和关键执行指标一致，作为重复方案过滤。"
            continue
        seen.add(signature)
        generated.append(scheme)
        if len(generated) >= 3:
            break
    return generated, attempted


def build_work_order_summary(data: Dict[str, Any]) -> Dict[str, Any]:
    work_order = data.get("work_order", {})
    return {
        "name": work_order.get("woker_order_name"),
        "guid": work_order.get("woker_order_guid"),
        "level": work_order.get("woker_order_level"),
        "execution": work_order.get("woker_order_execution"),
        "day_or_night": work_order.get("day_or_night"),
        "start_date": work_order.get("start_date"),
        "end_date": work_order.get("end_date"),
        "response_target_min": PARAMS["P012_response_min"],
        "target_count": len(parse_targets(data)),
    }


def _load_llm_client():
    try:
        from llm import llm_client
    except Exception:
        return None
    return llm_client


def _llm_text(content: Any) -> str:
    if content is None:
        return ""
    if isinstance(content, str):
        return content.strip()
    return str(content).strip()


def _extract_llm_output(resp: Any) -> str:
    if resp is None:
        return ""
    text = getattr(resp, "content", None)
    if text is None:
        text = str(resp)
    text = _llm_text(text)
    if text.startswith("```"):
        lines = text.splitlines()
        if len(lines) >= 3:
            text = "\n".join(lines[1:-1]).strip()
    return text


_NUMERIC_TOKEN_RE = re.compile(r"\d+(?:\.\d+)?")


def _numeric_tokens(text: str) -> List[str]:
    return _NUMERIC_TOKEN_RE.findall(text or "")


def _preserve_numeric_tokens(source: str, candidate: str) -> bool:
    return Counter(_numeric_tokens(source)) == Counter(_numeric_tokens(candidate))


_PROTECTED_TOKEN_RE = re.compile(r"(?<!\w)[A-Za-z0-9_-]{6,}(?!\w)")
_CONCLUSION_GROUPS = (
    ("不可行", "可行", "成功", "失败"),
    ("不可抢占", "可抢占", "允许", "禁止"),
    ("返航", "接力", "预警", "正常"),
)


def _conclusion_hits(text: str, group: Tuple[str, ...]) -> Tuple[str, ...]:
    """Match longer negative/status phrases before their shorter substrings."""
    hits = []
    remaining = text or ""
    for word in sorted(group, key=len, reverse=True):
        if word in remaining:
            hits.append(word)
            remaining = remaining.replace(word, "")
    return tuple(sorted(hits))


def _rewrite_is_safe(source: str, candidate: str) -> bool:
    """Reject rewrites that alter exact values, identifiers, or decision polarity."""
    if not candidate or not _preserve_numeric_tokens(source, candidate):
        return False
    for token in _PROTECTED_TOKEN_RE.findall(source):
        if token not in candidate:
            return False
    for group in _CONCLUSION_GROUPS:
        source_hits = _conclusion_hits(source, group)
        candidate_hits = _conclusion_hits(candidate, group)
        if source_hits != candidate_hits:
            return False
    return True


def _llm_facts(item: Any) -> Dict[str, Any]:
    """Keep prompts factual and small; route geometry is never sent for rewriting."""
    if not isinstance(item, dict):
        return {}
    facts = {}
    for key, value in item.items():
        if key in {"reason", "description", "suggestion", "rule", "trigger", "reject_reason"}:
            continue
        if isinstance(value, (str, int, float, bool)) or value is None:
            facts[key] = value
        elif isinstance(value, list) and all(isinstance(v, (str, int, float, bool)) for v in value):
            facts[key] = value
    return facts


def _rewrite_text(
    client: Any,
    text: str,
    context: str,
    label: str,
    facts: Optional[Dict[str, Any]] = None,
    cache: Optional[Dict[Tuple[str, str, str, str], str]] = None,
) -> str:
    if not client or not text:
        return text
    facts_text = json.dumps(facts or {}, ensure_ascii=False, sort_keys=True)
    cache_key = (text, context, label, facts_text)
    if cache is not None and cache_key in cache:
        return cache[cache_key]
    prompt = (
        f"你是无人机调度结果解释器。请在不改变事实、数字、专有名词和结论方向的前提下，"
        f"把下面这段{label}改写得更自然、更适合业务展示。\n"
        f"要求：只输出改写后的中文，不要解释，不要加前后缀，不要改 JSON 结构。\n"
        f"上下文：{context}\n"
        f"只读事实（不得新增、推断或修改）：{json.dumps(facts or {}, ensure_ascii=False)}\n"
        f"原文：{text}"
    )
    try:
        from langchain_core.messages import HumanMessage

        resp = client.invoke([HumanMessage(content=prompt)])
        new_text = _extract_llm_output(resp)
        if not new_text:
            if cache is not None:
                cache[cache_key] = text
            return text
        rewritten = new_text if _rewrite_is_safe(text, new_text) else text
        if cache is not None:
            cache[cache_key] = rewritten
        return rewritten
    except Exception:
        if cache is not None:
            cache[cache_key] = text
        return text


def _rewrite_text_items(
    client: Any,
    items: List[Dict[str, Any]],
    field_names: Tuple[str, ...],
    context: str,
    label: str,
    batch_size: int = 12,
) -> List[Dict[str, Any]]:
    if not client or not items:
        return items
    updated = [dict(item) for item in items]
    for start in range(0, len(updated), batch_size):
        batch = updated[start:start + batch_size]
        payload = []
        for idx, item in enumerate(batch):
            payload.append(
                {
                    "idx": idx,
                    "facts": _llm_facts(item),
                    **{field: item.get(field, "") for field in field_names},
                }
            )
        prompt = (
            f"你是无人机调度结果解释器。请只改写下面 JSON 数组中指定字段的中文说明，"
            f"不要改写任何数字、机场名、无人机ID、route_id、状态码，不要新增字段，不要删除字段，"
            f"不要改变条目数量，不要改变 idx。\n"
            f"输出必须是严格 JSON 数组，每个元素只包含 idx 和原字段，不要输出 facts。\n"
            f"上下文：{context}\n"
            f"字段：{', '.join(field_names)}\n"
            f"数据：{json.dumps(payload, ensure_ascii=False)}"
        )
        try:
            from langchain_core.messages import HumanMessage

            resp = client.invoke([HumanMessage(content=prompt)])
            raw = _extract_llm_output(resp)
            parsed = json.loads(raw)
            if not isinstance(parsed, list):
                continue
            parsed_map = {
                row.get("idx"): row
                for row in parsed
                if isinstance(row, dict) and "idx" in row
            }
            for idx, item in enumerate(batch):
                rewritten = parsed_map.get(idx)
                if not isinstance(rewritten, dict):
                    continue
                for field in field_names:
                    value = rewritten.get(field)
                    original = str(item.get(field, ""))
                    if isinstance(value, str) and value.strip() and _rewrite_is_safe(original, value):
                        item[field] = value.strip()
        except Exception:
            continue
    return updated


def enrich_result_with_llm(result: Dict[str, Any]) -> Dict[str, Any]:
    client = _load_llm_client()
    if not client:
        return result
    rewrite_cache: Dict[Tuple[str, str, str, str], str] = {}

    result["recommendation_reason"] = _rewrite_text(
        client,
        str(result.get("recommendation_reason", "")),
        "整份调度结果的推荐理由",
        "推荐理由",
        _llm_facts(result.get("summary")),
        rewrite_cache,
    )

    table_output = result.get("table_output") or {}
    decision_basis = table_output.get("decision_basis", {}).get("data", {})
    if isinstance(decision_basis, dict):
        decision_basis["reason"] = _rewrite_text(
            client,
            str(decision_basis.get("reason", "")),
            "最终推荐方案的决策依据",
            "决策依据",
            _llm_facts(result.get("summary")),
            rewrite_cache,
        )

    candidate_set = table_output.get("candidate_airport_set", {}).get("data", [])
    if isinstance(candidate_set, list):
        table_output["candidate_airport_set"]["data"] = _rewrite_text_items(
            client,
            candidate_set,
            ("reason",),
            "候选机场集合中的每条候选说明",
            "候选机场说明",
        )

    compliant_set = table_output.get("compliant_airport_set", {}).get("data", [])
    if isinstance(compliant_set, list):
        table_output["compliant_airport_set"]["data"] = _rewrite_text_items(
            client,
            compliant_set,
            ("reason",),
            "合规机场集合中的每条合规说明",
            "合规机场说明",
        )

    scheme_options = table_output.get("scheme_options", {}).get("data", [])
    if isinstance(scheme_options, list):
        for row in scheme_options:
            if isinstance(row, dict):
                row["description"] = _rewrite_text(
                    client,
                    str(row.get("description", "")),
                    f"方案 {row.get('scheme_label', '')} 的对比说明",
                    "方案说明",
                    _llm_facts(row),
                    rewrite_cache,
                )

    infeasible_items = table_output.get("infeasible_reason", {}).get("data", [])
    if isinstance(infeasible_items, list):
        for item in infeasible_items:
            if not isinstance(item, dict):
                continue
            item["reason"] = _rewrite_text(
                client,
                str(item.get("reason", "")),
                f"不可行方案 {item.get('scheme_label', '')} 的失败原因",
                "失败原因",
                _llm_facts(item),
                rewrite_cache,
            )
            item["suggestion"] = _rewrite_text(
                client,
                str(item.get("suggestion", "")),
                f"不可行方案 {item.get('scheme_label', '')} 的处理建议",
                "处理建议",
                _llm_facts(item),
                rewrite_cache,
            )
            failed_plans = item.get("failed_plans", [])
            if isinstance(failed_plans, list):
                item["failed_plans"] = _rewrite_text_items(
                    client,
                    failed_plans,
                    ("reason",),
                    f"不可行方案 {item.get('scheme_label', '')} 的失败段说明",
                    "失败段说明",
                )

    preempt_relay = table_output.get("preempt_relay_cooperation", {}).get("data", {})
    if isinstance(preempt_relay, dict):
        preempt = preempt_relay.get("preempt", {})
        if isinstance(preempt, dict):
            preempt["rule"] = _rewrite_text(
                client,
                str(preempt.get("rule", "")),
                "抢占规则说明",
                "抢占规则",
                _llm_facts(preempt),
                rewrite_cache,
            )
        relay_rows = preempt_relay.get("relay", [])
        if isinstance(relay_rows, list):
            preempt_relay["relay"] = _rewrite_text_items(
                client,
                relay_rows,
                ("reason",),
                "接力决策说明",
                "接力说明",
            )
        cooperation = preempt_relay.get("cooperation", {})
        if isinstance(cooperation, dict):
            cooperation["trigger"] = _rewrite_text(
                client,
                str(cooperation.get("trigger", "")),
                "协同触发条件说明",
                "协同触发",
                    _llm_facts(cooperation),
                    rewrite_cache,
            )

    attempted = result.get("attempted_schemes", [])
    if isinstance(attempted, list):
        for scheme in attempted:
            if not isinstance(scheme, dict):
                continue
            if "reject_reason" in scheme:
                scheme["reject_reason"] = _rewrite_text(
                    client,
                    str(scheme.get("reject_reason", "")),
                    f"策略 {scheme.get('scheme_label', '')} 的筛除原因",
                    "筛除原因",
                    _llm_facts(scheme),
                    rewrite_cache,
                )

    result["table_output"] = table_output
    return result


def _join_names(items: List[Dict[str, Any]], key: str = "name") -> str:
    names = []
    for item in items:
        name = str(item.get(key, "")).strip()
        if name and name not in names:
            names.append(name)
    return "、".join(names) if names else "-"


def _uniq_sorted(values: List[str]) -> str:
    uniq = []
    for value in values:
        if value and value not in uniq:
            uniq.append(value)
    return "、".join(uniq) if uniq else "-"


def _unique_airports_from_candidates(rows: List[Dict[str, Any]], flag: str) -> List[Dict[str, Any]]:
    airports = {}
    for row in rows:
        for candidate in row.get("candidates", []):
            if not candidate.get(flag):
                continue
            uid = candidate.get("airport_uid")
            if uid in airports:
                continue
            airports[uid] = {
                "airport_uid": uid,
                "airport_name": candidate.get("airport_name"),
                "inspection_radius_m": candidate.get("inspection_radius_m"),
                "distance_m": candidate.get("distance_m"),
                "reason": candidate.get("reason"),
            }
    return list(airports.values())


def _assignment_rows(scheme: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = []
    for row in scheme.get("assignments", []):
        rows.append(
            {
                "work_object": row.get("target"),
                "route_id": row.get("route_id"),
                "airport_uid": row.get("assigned_airport_uid"),
                "airport_name": row.get("assigned_airport_name"),
                "drone_id": row.get("drone_id"),
                "inspected_object_count": 1,
                "waypoint_count": row.get("waypoint_count"),
                "distance_m": row.get("distance_m"),
                "flight_min": row.get("flight_min"),
                "work_min": row.get("work_min"),
                "prepare_min": PARAMS["P040_prepare_min"],
                "safety_min": PARAMS["P011_safety_fixed_min"],
                "total_min": row.get("total_min"),
                "battery_use_pct": row.get("battery_use_pct"),
                "planned_takeoff_offset_min": row.get("start_offset_min"),
                "planned_finish_offset_min": row.get("end_offset_min"),
            }
        )
    return rows


def _sortie_duration_rows(scheme: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = []
    for plan in scheme.get("plans", []):
        route_ids = [r["obj_id"] for r in plan.get("route", [])]
        rows.append(
            {
                "airport_uid": plan.get("airport_uid"),
                "airport_name": plan.get("airport_name"),
                "drone_id": plan.get("drone_id"),
                "sortie_index": plan.get("sortie_index"),
                "relay_leg": plan.get("relay_leg"),
                "relay_legs": plan.get("relay_legs"),
                "route_ids": route_ids,
                "flight_min": plan.get("flight_min"),
                "work_min": plan.get("work_min"),
                "prepare_min": plan.get("prepare_min"),
                "safety_min": plan.get("safety_min"),
                "total_min": plan.get("total_min"),
                "battery_use_pct": plan.get("battery_use_pct"),
                "battery_remaining_pct": plan.get("battery_remaining_pct"),
                "battery_level": plan.get("battery_level"),
                "feasible": plan.get("feasible_after_endurance_check"),
                "segment_index": plan.get("segment_index"),
                "segment_count": plan.get("segment_count"),
                "segment_start_waypoint": plan.get("segment_start_waypoint"),
                "segment_end_waypoint": plan.get("segment_end_waypoint"),
                "inspected_object_count": len(route_ids),
                "planned_takeoff_offset_min": plan.get("planned_takeoff_offset_min"),
                "planned_finish_offset_min": plan.get("planned_finish_offset_min"),
                "charge_wait_before_takeoff_min": plan.get("charge_wait_before_takeoff_min"),
                "charging_duration_min": plan.get("charging_duration_min"),
                "resource_recovery_end_offset_min": plan.get("resource_recovery_end_offset_min"),
                "resource_recovery_end_time": plan.get("resource_recovery_end_time"),
                "planned_start_time": plan.get("planned_start_time"),
                "planned_end_time": plan.get("planned_end_time"),
                "within_work_order_window": plan.get("within_work_order_window"),
            }
        )
    return rows


def _route_rows_for_scheme(scheme: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = []
    for plan in scheme.get("plans", []):
        for route in plan.get("route", []):
            waypoints = []
            for index, point in enumerate(route.get("waypoints", []), start=1):
                waypoints.append(
                    {
                        "seq": index,
                        "name": point.get("name"),
                        "lon": point.get("lon"),
                        "lat": point.get("lat"),
                        "altitude": point.get("altitude"),
                    }
                )
            flight_path = [
                {
                    "seq": 0,
                    "type": "takeoff_airport",
                    "name": plan.get("airport_name"),
                    "airport_uid": plan.get("airport_uid"),
                }
            ]
            flight_path.extend(
                {
                    "seq": wp["seq"],
                    "type": "waypoint",
                    "name": wp["name"],
                    "lon": wp["lon"],
                    "lat": wp["lat"],
                    "altitude": wp["altitude"],
                }
                for wp in waypoints
            )
            flight_path.append(
                {
                    "seq": len(waypoints) + 1,
                    "type": "return_airport",
                    "name": plan.get("airport_name"),
                    "airport_uid": plan.get("airport_uid"),
                }
            )
            rows.append(
                {
                    "route_id": route.get("obj_id"),
                    "route_name": route.get("obj_name"),
                    "inspected_object_count": 1,
                    "airport_uid": plan.get("airport_uid"),
                    "airport_name": plan.get("airport_name"),
                    "airport_lon": plan.get("airport_lon"),
                    "airport_lat": plan.get("airport_lat"),
                    "drone_id": plan.get("drone_id"),
                    "sortie_index": plan.get("sortie_index"),
                    "relay_leg": plan.get("relay_leg"),
                    "relay_legs": plan.get("relay_legs"),
                    "planned_start_time": plan.get("planned_start_time"),
                    "planned_end_time": plan.get("planned_end_time"),
                    "charge_wait_before_takeoff_min": plan.get("charge_wait_before_takeoff_min"),
                    "charging_duration_min": plan.get("charging_duration_min"),
                    "resource_recovery_end_time": plan.get("resource_recovery_end_time"),
                    "within_work_order_window": plan.get("within_work_order_window"),
                    "segment_index": plan.get("segment_index"),
                    "segment_count": plan.get("segment_count"),
                    "segment_start_waypoint": plan.get("segment_start_waypoint"),
                    "segment_end_waypoint": plan.get("segment_end_waypoint"),
                    "waypoint_count": len(waypoints),
                    "first_waypoint": route.get("first_waypoint"),
                    "last_waypoint": route.get("last_waypoint"),
                    "waypoints": waypoints,
                    "flight_path": flight_path,
                    "flight_rule": "从机场起飞，按waypoints顺序巡检，完成本接力段后返航；如relay_legs>1，则返航后换电/充电再执行下一接力段。",
                }
            )
    return rows


def _sortie_distribution_rows(scheme: Dict[str, Any]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, int], Dict[str, Any]] = {}
    for plan in scheme.get("plans", []):
        key = (plan.get("airport_uid"), plan.get("sortie_index"))
        row = grouped.setdefault(
            key,
            {
                "airport_uid": plan.get("airport_uid"),
                "airport_name": plan.get("airport_name"),
                "drone_id": plan.get("drone_id"),
                "sortie_index": plan.get("sortie_index"),
                "relay_legs": plan.get("relay_legs"),
                "relay_point": plan.get("airport_name"),
                "relay_point_type": "airport_return_and_takeoff",
                "route_ids": [],
                "route_names": [],
                "inspected_object_count": 0,
                "total_waypoint_count": 0,
                "relay_segments": [],
            },
        )
        for route in plan.get("route", []):
            route_id = route.get("obj_id")
            if route_id not in row["route_ids"]:
                row["route_ids"].append(route_id)
                row["route_names"].append(route.get("obj_name"))
                row["inspected_object_count"] += 1
            row["total_waypoint_count"] += int(route.get("waypoint_count", 0) or 0)
            row["relay_segments"].append(
                {
                    "relay_leg": plan.get("relay_leg"),
                    "relay_legs": plan.get("relay_legs"),
                    "segment_index": plan.get("segment_index"),
                    "segment_count": plan.get("segment_count"),
                    "route_id": route_id,
                    "route_name": route.get("obj_name"),
                    "start_waypoint": route.get("first_waypoint"),
                    "end_waypoint": route.get("last_waypoint"),
                    "waypoint_count": route.get("waypoint_count"),
                }
            )
    return list(grouped.values())


def _scheme_option_rows(schemes: List[Dict[str, Any]], recommended: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    recommended_name = recommended.get("scheme_name") if recommended else None
    rows = []
    for scheme in schemes:
        s = scheme["summary"]
        rows.append(
            {
                "scheme_name": scheme["scheme_name"],
                "scheme_label": scheme["scheme_label"],
                "description": scheme["description"],
                "is_recommended": scheme["scheme_name"] == recommended_name,
                "status": scheme["status"],
                "assigned_target_count": s["assigned_target_count"],
                "segment_count": s["segment_count"],
                "used_airport_count": s["used_airport_count"],
                "sortie_count": s["sortie_count"],
                "total_route_distance_m": s["total_route_distance_m"],
                "max_plan_duration_min": s["max_plan_duration_min"],
                "min_battery_remaining_pct": s["min_battery_remaining_pct"],
                "warn_or_worse_count": s["warn_or_worse_count"],
                "estimated_completion_min": s.get("estimated_completion_min", 0),
                "sortie_distribution": _sortie_distribution_rows(scheme),
                "routes": _route_rows_for_scheme(scheme),
            }
        )
    return rows


def build_table_output(result: Dict[str, Any]) -> Dict[str, Any]:
    recommended = result.get("recommended_scheme")
    source_scheme = recommended or (result.get("attempted_schemes") or [{}])[0]
    candidate_rows = []
    for item in source_scheme.get("unassigned", []):
        candidate_rows.append(item)
    for item in source_scheme.get("decision_chain", []):
        candidate_rows.append({"candidates": item.get("candidate_airports", [])})

    candidate_airports = _unique_airports_from_candidates(candidate_rows, "in_coverage")
    compliant_airports = _unique_airports_from_candidates(candidate_rows, "compliant")
    assignments = _assignment_rows(source_scheme) if recommended else []
    sortie_durations = _sortie_duration_rows(source_scheme) if recommended else []
    route_ids = [
        {
            "route_id": row["route_id"],
            "work_object": row["work_object"],
            "airport_uid": row["airport_uid"],
            "airport_name": row["airport_name"],
            "route_type": "temporary_planned",
        }
        for row in assignments
    ]
    infeasible_reasons = []
    for scheme in result.get("attempted_schemes", []):
        if scheme["status"] != "success" or scheme.get("reject_reason"):
            failed_plans = []
            for plan in scheme.get("plans", []):
                if plan.get("feasible_after_endurance_check"):
                    continue
                battery_remaining_pct = plan.get("battery_remaining_pct")
                max_allowed_min = plan.get("max_allowed_min")
                total_min = plan.get("total_min")
                battery_level = plan.get("battery_level")
                if battery_level == "返航":
                    detail_reason = (
                        f"单段总时长{total_min}min已超出允许上限{max_allowed_min}min，"
                        f"剩余电量{battery_remaining_pct}%进入返航风险。"
                    )
                elif battery_level == "接力":
                    detail_reason = (
                        f"单段总时长{total_min}min已超出允许上限{max_allowed_min}min，"
                        f"剩余电量{battery_remaining_pct}%已到接力线。"
                    )
                elif battery_level == "预警":
                    detail_reason = (
                        f"单段总时长{total_min}min已超出允许上限{max_allowed_min}min，"
                        f"剩余电量{battery_remaining_pct}%处于预警区。"
                    )
                else:
                    detail_reason = (
                        f"单段总时长{total_min}min已超出允许上限{max_allowed_min}min，"
                        f"剩余电量{battery_remaining_pct}%。"
                    )
                failed_plans.append(
                    {
                        "airport_name": plan.get("airport_name"),
                        "drone_id": plan.get("drone_id"),
                        "sortie_index": plan.get("sortie_index"),
                        "relay_leg": plan.get("relay_leg"),
                        "relay_legs": plan.get("relay_legs"),
                        "route_id": plan.get("route", [{}])[0].get("obj_id"),
                        "segment_start_waypoint": plan.get("segment_start_waypoint"),
                        "segment_end_waypoint": plan.get("segment_end_waypoint"),
                        "battery_remaining_pct": battery_remaining_pct,
                        "battery_level": battery_level,
                        "total_min": total_min,
                        "max_allowed_min": max_allowed_min,
                        "reason": detail_reason,
                    }
                )
            infeasible_reasons.append(
                {
                    "scheme_name": scheme["scheme_name"],
                    "scheme_label": scheme["scheme_label"],
                    "status": scheme["status"],
                    "reason": scheme.get("reject_reason", "未完整通过方案筛选"),
                    "failed_plans": failed_plans,
                    "unassigned": scheme.get("unassigned", []),
                    "suggestion": "检查覆盖半径、机场/无人机状态、气象阈值和电量阈值；必要时进入延期池或人工改派。",
                }
            )
    relay_rows = []
    if recommended:
        for plan in recommended.get("plans", []):
            if plan.get("relay_legs", 1) > 1:
                relay_rows.append(
                    {
                        "decision_type": "relay",
                        "airport_uid": plan.get("airport_uid"),
                        "airport_name": plan.get("airport_name"),
                        "drone_id": plan.get("drone_id"),
                        "sortie_index": plan.get("sortie_index"),
                        "relay_leg": plan.get("relay_leg"),
                        "relay_legs": plan.get("relay_legs"),
                        "reason": plan.get("relay_reason"),
                        "charging_duration_min": plan.get("charging_duration_min"),
                    }
                )
    cooperation_enabled = result["work_order"]["target_count"] >= PARAMS["P029_multi_object_trigger"]
    cooperation_airports = len({row["airport_uid"] for row in assignments}) if assignments else 0
    return {
        "candidate_airport_set": {
            "index": 1,
            "name": "候选机场集合",
            "meaning": "覆盖矩阵预筛结果（不在覆盖列表的机场不参与）",
            "next_step": "决策链后续步骤的输入",
            "data": candidate_airports,
        },
        "compliant_airport_set": {
            "index": 2,
            "name": "合规机场集合",
            "meaning": "硬约束剔除后（空域/气象/设备状态）",
            "next_step": "距离排序输入",
            "data": compliant_airports,
        },
        "assignment_result": {
            "index": 3,
            "name": "指派结果（作业对象→机场+无人机）",
            "meaning": "就近 + 覆盖 + 续航校验 的最终匹配",
            "next_step": "方案核心",
            "data": assignments,
        },
        "schedule_timeline": {
            "index": 4,
            "name": "准备开始时间 / 预计完成时间",
            "meaning": f"响应锚点顺排（P012，首段准备开始锚点={PARAMS['P012_response_min']}min）",
            "next_step": "方案时间轴",
            "data": [
                {
                    "airport_uid": row["airport_uid"],
                    "airport_name": row["airport_name"],
                    "drone_id": row["drone_id"],
                    "sortie_index": row["sortie_index"],
                    "relay_leg": row["relay_leg"],
                    "relay_legs": row["relay_legs"],
                    "route_ids": row["route_ids"],
                    "inspected_object_count": row["inspected_object_count"],
                    "segment_start_waypoint": row["segment_start_waypoint"],
                    "segment_end_waypoint": row["segment_end_waypoint"],
                    "planned_takeoff_offset_min": row["planned_takeoff_offset_min"],
                    "planned_finish_offset_min": row["planned_finish_offset_min"],
                    "charge_wait_before_takeoff_min": row["charge_wait_before_takeoff_min"],
                    "charging_duration_min": row["charging_duration_min"],
                    "resource_recovery_end_offset_min": row["resource_recovery_end_offset_min"],
                    "resource_recovery_end_time": row["resource_recovery_end_time"],
                    "planned_start_time": row["planned_start_time"],
                    "planned_end_time": row["planned_end_time"],
                    "within_work_order_window": row["within_work_order_window"],
                    "response_target_min": result["work_order"]["response_target_min"],
                }
                for row in sortie_durations
            ],
        },
        "route_ids": {
            "index": 5,
            "name": "航线 route_id（预存/临时规划）",
            "meaning": "航线库匹配结果",
            "next_step": "平台下发执飞",
            "data": route_ids,
        },
        "sortie_duration": {
            "index": 6,
            "name": "单架次飞行/作业/总时长",
            "meaning": f"距离÷速度 + 航点数×{PARAMS['P039_work_sec_per_waypoint']}秒 + 准备{PARAMS['P040_prepare_min']}min + 冗余{PARAMS['P011_safety_fixed_min']}min",
            "next_step": "续航校验、展示",
            "data": sortie_durations,
        },
        "scheme_options": {
            "index": 7,
            "name": "三套方案 + 推荐",
            "meaning": "时效最优 / 资源最优 / 稳健均衡",
            "next_step": "人工确认",
            "data": _scheme_option_rows(result.get("schemes", []), recommended),
        },
        "decision_basis": {
            "index": 8,
            "name": "决策依据",
            "meaning": "为什么选这架（可解释）",
            "next_step": "展示、留痕",
            "data": {
                "recommended_scheme": recommended["scheme_name"] if recommended else None,
                "recommended_label": recommended["scheme_label"] if recommended else None,
                "reason": result.get("recommendation_reason"),
            },
        },
        "infeasible_reason": {
            "index": 9,
            "name": "不可行原因 InfeasibleReason",
            "meaning": "无解时：冲突约束 + 建议",
            "next_step": "拦截 / 进延期池",
            "data": infeasible_reasons,
        },
        "preempt_relay_cooperation": {
            "index": 10,
            "name": "抢占 / 接力 / 协同决策",
            "meaning": "1级终止≥2级 / 电量接力 / 多机并行",
            "next_step": "执行指令",
            "data": {
                "preempt": {
                    "enabled": int(result["work_order"]["level"] or 4) == PARAMS["P013_preempt_incoming_level"],
                    "rule": "1级工单仅可抢占未到进度保护线的低优先级任务，不可抢占同为1级的任务",
                },
                "relay": relay_rows,
                "cooperation": {
                    "enabled": cooperation_enabled,
                    "trigger": f"对象数>={PARAMS['P029_multi_object_trigger']}触发多机场/多机协同评估",
                    "used_airport_count": cooperation_airports,
                    "is_multi_airport": cooperation_airports > 1,
                },
            },
        },
    }


def _route_color(index: int) -> Tuple[int, int, int]:
    palette = [
        (70, 130, 180),
        (220, 90, 70),
        (60, 160, 110),
        (145, 100, 205),
        (220, 150, 40),
        (90, 90, 90),
    ]
    return palette[index % len(palette)]


def _bbox(points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    lons = [p[0] for p in points]
    lats = [p[1] for p in points]
    return min(lons), min(lats), max(lons), max(lats)


def _draw_arrow(
    draw: ImageDraw.ImageDraw,
    p1: Tuple[int, int],
    p2: Tuple[int, int],
    color: Tuple[int, int, int],
    width: int = 4,
) -> None:
    draw.line((p1, p2), fill=color, width=width)
    angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    head = 14 + width
    left = (
        p2[0] - head * math.cos(angle - math.pi / 6),
        p2[1] - head * math.sin(angle - math.pi / 6),
    )
    right = (
        p2[0] - head * math.cos(angle + math.pi / 6),
        p2[1] - head * math.sin(angle + math.pi / 6),
    )
    draw.polygon([p2, left, right], fill=color)


def _draw_dashed_line(
    draw: ImageDraw.ImageDraw,
    p1: Tuple[int, int],
    p2: Tuple[int, int],
    color: Tuple[int, int, int],
    width: int = 3,
    dash: int = 12,
) -> None:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = math.hypot(dx, dy)
    if dist <= 0:
        return
    steps = max(int(dist / dash), 1)
    for i in range(0, steps, 2):
        start = i / steps
        end = min((i + 1) / steps, 1)
        a = (int(p1[0] + dx * start), int(p1[1] + dy * start))
        b = (int(p1[0] + dx * end), int(p1[1] + dy * end))
        draw.line((a, b), fill=color, width=width)


def _short_airport_name(name: str) -> str:
    for suffix in ("实飞A", "实飞B", "实飞C", "实飞D"):
        if suffix in name:
            return suffix.replace("实飞", "") + "机场"
    return name[-6:] if len(name) > 6 else name


def render_scheme_diagram(scheme: Dict[str, Any], out_path: str) -> str:
    width, height = 1650, 1000
    margin = 90
    legend_width = 430
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    font_path = "/System/Library/Fonts/STHeiti Medium.ttc"
    try:
        font = ImageFont.truetype(font_path, 18)
        title_font = ImageFont.truetype(font_path, 24)
    except OSError:
        font = ImageFont.load_default()
        title_font = ImageFont.load_default()

    airports = {}
    points: List[Tuple[float, float]] = []
    for plan in scheme.get("plans", []):
        airports[plan["airport_uid"]] = {
            "name": plan["airport_name"],
            "lon": float(plan.get("airport_lon", 0) or 0),
            "lat": float(plan.get("airport_lat", 0) or 0),
        }
        points.append((float(plan.get("airport_lon", 0) or 0), float(plan.get("airport_lat", 0) or 0)))
        for route in plan.get("route", []):
            for wp in route.get("waypoints", []):
                points.append((float(wp.get("lon", 0) or 0), float(wp.get("lat", 0) or 0)))
    if not points:
        points = [(0.0, 0.0), (1.0, 1.0)]
    min_lon, min_lat, max_lon, max_lat = _bbox(points)
    if abs(max_lon - min_lon) < 1e-9:
        max_lon += 0.01
        min_lon -= 0.01
    if abs(max_lat - min_lat) < 1e-9:
        max_lat += 0.01
        min_lat -= 0.01

    def transform(lon: float, lat: float) -> Tuple[int, int]:
        plot_width = width - legend_width - 2 * margin
        x = margin + (lon - min_lon) / (max_lon - min_lon) * plot_width
        y = height - margin - (lat - min_lat) / (max_lat - min_lat) * (height - 2 * margin)
        return int(x), int(y)

    # Background grid
    for i in range(1, 6):
        x = margin + i * (width - legend_width - 2 * margin) / 6
        y = margin + i * (height - 2 * margin) / 6
        draw.line((x, margin, x, height - margin), fill=(235, 235, 235), width=1)
        draw.line((margin, y, width - legend_width - margin, y), fill=(235, 235, 235), width=1)

    draw.rounded_rectangle((20, 20, width - 20, height - 20), radius=18, outline=(180, 180, 180), width=2)
    draw.text((40, 35), f"方案草图: {scheme['scheme_label']} ({scheme['scheme_name']})", fill=(20, 20, 20), font=title_font)
    s = scheme["summary"]
    draw.text(
        (40, 64),
        f"架次{s['sortie_count']} | 机场{s['used_airport_count']} | 最长{s['max_plan_duration_min']}min | 最低余量{s['min_battery_remaining_pct']}%",
        fill=(90, 90, 90),
        font=font,
    )

    airport_groups: Dict[str, Dict[str, Any]] = {}
    for plan in scheme.get("plans", []):
        group = airport_groups.setdefault(
            plan["airport_uid"],
            {
                "airport": {
                    "uid": plan["airport_uid"],
                    "name": plan["airport_name"],
                    "lon": float(plan.get("airport_lon", 0) or 0),
                    "lat": float(plan.get("airport_lat", 0) or 0),
                },
                "routes": [],
                "relay_legs": [],
                "sortie_indexes": set(),
            },
        )
        group["routes"].extend(plan.get("route", []))
        group["relay_legs"].append(plan.get("relay_legs", 1))
        group["sortie_indexes"].add(plan.get("sortie_index"))

    airport_items = list(airport_groups.values())
    legend_rows = []
    sortie_no = 1
    for idx, group in enumerate(airport_items):
        color = _route_color(idx)
        airport = group["airport"]
        airport_pt = transform(airport["lon"], airport["lat"])
        sortie_groups: Dict[int, Dict[str, Any]] = {}
        for plan in scheme.get("plans", []):
            if plan["airport_uid"] != airport["uid"]:
                continue
            sortie_groups.setdefault(
                plan["sortie_index"],
                {
                    "plans": [],
                },
            )["plans"].append(plan)
        for sortie_index in sorted(sortie_groups):
            group_data = sortie_groups[sortie_index]
            for plan in sorted(group_data["plans"], key=lambda p: p.get("relay_leg", 1)):
                route_objects = plan.get("route", [])
                if not route_objects:
                    continue
                path_points = [airport_pt]
                sortie_route_names = []
                waypoint_total = 0
                for route in route_objects:
                    waypoints = route.get("waypoints", [])
                    if not waypoints:
                        continue
                    sortie_route_names.append(route.get("obj_id", ""))
                    waypoint_total += len(waypoints)
                    path_points.extend(
                        transform(float(wp.get("lon", 0) or 0), float(wp.get("lat", 0) or 0))
                        for wp in waypoints
                    )
                if len(path_points) < 2:
                    continue
                path_points.append(airport_pt)
                outbound_color = (150, 150, 150)
                _draw_arrow(draw, airport_pt, path_points[1], outbound_color, width=2)
                _draw_dashed_line(draw, path_points[-2], airport_pt, outbound_color, width=3)
                mid_x = int((path_points[-2][0] + airport_pt[0]) / 2)
                mid_y = int((path_points[-2][1] + airport_pt[1]) / 2)
                draw.text((mid_x + 6, mid_y - 18), "返航接力", fill=(120, 120, 120), font=font)
                draw.line(path_points, fill=color, width=3)
                arrow_step = max(len(path_points) // 4, 1)
                for pos in range(1, len(path_points) - 2, arrow_step):
                    _draw_arrow(draw, path_points[pos], path_points[pos + 1], color, width=3)
                label_pt = path_points[len(path_points) // 2]
                short_label = f"S{sortie_no}"
                if plan.get("relay_legs", 1) > 1:
                    short_label = f"S{sortie_no}-L{plan.get('relay_leg')}"
                draw.rounded_rectangle(
                    (label_pt[0] - 34, label_pt[1] - 15, label_pt[0] + 34, label_pt[1] + 15),
                    radius=8,
                    fill=(255, 255, 255),
                    outline=color,
                    width=2,
                )
                draw.text((label_pt[0] - 28, label_pt[1] - 11), short_label, fill=color, font=font)
                route_count = len({r.get("obj_id") for r in route_objects})
                draw.text(
                    (label_pt[0] + 40, label_pt[1] - 14),
                    f"{route_count}对象 / 第{plan.get('relay_leg')}/{plan.get('relay_legs')}段",
                    fill=color,
                    font=font,
                )
                route_ids_text = "、".join(sortie_route_names) if sortie_route_names else "-"
                legend_rows.append(
                    {
                        "label": short_label,
                        "name": (
                            f"{_short_airport_name(airport['name'])} "
                            f"架次{sortie_index} 第{plan.get('relay_leg')}/{plan.get('relay_legs')}段 "
                            f"巡检{route_count}对象 / {waypoint_total}航点"
                        ),
                        "route_id": route_ids_text,
                        "airport": airport["name"],
                        "color": color,
                    }
                )
            sortie_no += 1
        summary_text = f"架次{len(group['sortie_indexes'])} / 接力{'、'.join(str(x) for x in sorted(set(group['relay_legs'])))}段"
        draw.text((airport_pt[0] + 10, airport_pt[1] + 8), summary_text, fill=color, font=font)

    drawn_airports = set()
    for group in airport_items:
        airport = group["airport"]
        airport_uid = airport["uid"]
        if airport_uid in drawn_airports:
            continue
        drawn_airports.add(airport_uid)
        color = (35, 35, 35)
        x, y = transform(airport["lon"], airport["lat"])
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill=(255, 255, 255), outline=color, width=3)
        draw.text((x + 10, y - 16), airport["name"], fill=color, font=font)
        draw.text((x + 10, y + 6), "接力点/换电点", fill=(90, 90, 90), font=font)

    legend_x = width - legend_width + 20
    legend_y = 38
    draw.rounded_rectangle(
        (legend_x - 12, legend_y - 8, width - 40, height - 40),
        radius=10,
        fill=(250, 250, 250),
        outline=(205, 205, 205),
        width=1,
    )
    draw.text((legend_x, legend_y), "图例", fill=(30, 30, 30), font=title_font)
    draw.text((legend_x, legend_y + 36), "圆点：机场 / 接力点 / 换电点", fill=(40, 40, 40), font=font)
    draw.text((legend_x, legend_y + 62), "灰色箭头：机场飞到线路", fill=(40, 40, 40), font=font)
    draw.text((legend_x, legend_y + 88), "彩色箭头：线路巡检方向", fill=(40, 40, 40), font=font)
    draw.text((legend_x, legend_y + 114), "灰色虚线：巡检后返航", fill=(40, 40, 40), font=font)
    draw.text((legend_x, legend_y + 150), "线路编号", fill=(30, 30, 30), font=title_font)
    y = legend_y + 188
    for row in legend_rows[:18]:
        color = row["color"]
        draw.line((legend_x, y + 10, legend_x + 28, y + 10), fill=color, width=6)
        route_short = row["route_id"]
        if len(route_short) > 20:
            route_short = f"{route_short[:6]}…{route_short[-4:]}"
        draw.multiline_text(
            (legend_x + 38, y - 2),
            f"{row['label']} {row['name']}\n{route_short}",
            fill=(40, 40, 40),
            font=font,
            spacing=3,
        )
        y += 42

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    img.save(out_path)
    return out_path


def render_scheme_diagrams(result: Dict[str, Any], out_dir: str) -> None:
    if not result.get("schemes"):
        return
    os.makedirs(out_dir, exist_ok=True)
    for scheme in result["schemes"]:
        path = os.path.join(out_dir, f"core_demo_{scheme['scheme_name']}.png")
        render_scheme_diagram(scheme, path)
        for row in result["table_output"]["scheme_options"]["data"]:
            if row["scheme_name"] == scheme["scheme_name"]:
                row["diagram_path"] = path
                row["diagram_name"] = os.path.basename(path)
                break


def empty_summary(data: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "airport_count": len(parse_airports(data)),
        "assigned_target_count": 0,
        "unassigned_target_count": len(parse_targets(data)),
        "used_airport_count": 0,
        "segment_count": 0,
        "sortie_count": 0,
        "total_route_distance_m": 0,
        "max_plan_duration_min": 0,
        "min_battery_remaining_pct": 0,
        "avg_battery_remaining_pct": 0,
        "warn_or_worse_count": 0,
        "return_risk_count": 0,
    }


def solve(data: Dict[str, Any]) -> Dict[str, Any]:
    apply_runtime_params(data)
    work_order = data.get("work_order", {})
    schemes, attempted_schemes = generate_schemes(data)
    if not schemes:
        empty_result = {
            "status": "infeasible",
            "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "work_order": build_work_order_summary(data),
            "summary": empty_summary(data),
            "schemes": [],
            "attempted_schemes": attempted_schemes,
            "recommended_scheme": None,
            "recommendation_reason": "没有生成可执行方案：三种策略均未完整通过覆盖、资源可用性和续航安全校验，或被判定为重复执行方案。",
            "rule_audit": build_rule_audit(data),
            "assignments": [],
            "decision_chain": [],
            "unassigned": [
                {"target": t.name, "reason": "未生成可执行方案"}
                for t in parse_targets(data)
            ],
        }
        empty_result["table_output"] = build_table_output(empty_result)
        return empty_result
    recommended = min(schemes, key=lambda s: recommendation_key(s, work_order))
    recommendation_reason = build_recommendation_reason(recommended, schemes, work_order)
    result = {
        "status": recommended["status"] if recommended["status"] == "success" else "partial",
        "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "work_order": recommended["work_order"],
        "summary": recommended["summary"],
        "schemes": schemes,
        "attempted_schemes": attempted_schemes,
        "recommended_scheme": recommended,
        "recommendation_reason": recommendation_reason,
        "rule_audit": build_rule_audit(data),
        "assignments": recommended["assignments"],
        "decision_chain": recommended["decision_chain"],
        "unassigned": recommended["unassigned"],
    }
    result["table_output"] = build_table_output(result)
    return result


def print_summary(result: Dict[str, Any]) -> None:
    wo = result["work_order"]
    summary = result["summary"]
    print("=== 无人机核心调度 Demo ===")
    print(f"工单: {wo['name']} | 等级: {wo['level']} | 对象数: {wo['target_count']}")
    print(f"执行方式: {wo['execution']} | 时段要求: {wo.get('day_or_night')} | 响应目标: {wo['response_target_min']}min")
    print(f"状态: {result['status']}")
    rec = result["recommended_scheme"]
    print(f"生成方案数: {len(result['schemes'])}/最多3套")
    if rec is None:
        print("推荐方案: 无")
    else:
        print(f"推荐方案: {rec['scheme_label']} ({rec['scheme_name']})")
    print(f"推荐理由: {result['recommendation_reason']}")
    print(
        "分配: "
        f"{summary['assigned_target_count']} 已分配 / {summary['unassigned_target_count']} 未分配, "
        f"使用机场 {summary['used_airport_count']} 个"
    )
    print(f"总航线距离: {summary['total_route_distance_m']} m")
    print(f"最长单机场任务耗时: {summary['max_plan_duration_min']} min")
    print(
        f"预计整体完成: {summary.get('estimated_completion_min', 0)} min "
        f"({summary.get('estimated_start_time', '')} -> {summary.get('estimated_completion_time', '')})"
    )
    print("")
    print("决策链:")
    decision_chain = result.get("decision_chain", [])
    if decision_chain:
        candidate_airports = _uniq_sorted(
            [x.get("airport_name", "") for row in decision_chain for x in row.get("candidate_airports", [])]
        )
        compliant_airports = _uniq_sorted(
            [x.get("airport_name", "") for row in decision_chain for x in row.get("compliant_airports", [])]
        )
        route_ids = _uniq_sorted([row.get("route_id", "") for row in decision_chain])
        assigned_pairs = "；".join(
            f"{row['target']}→{row['airport_name']} / {row['drone_id']}" for row in decision_chain
        )
        print(f"1. 候选机场集合: {candidate_airports}")
        print(f"2. 合规机场集合: {compliant_airports}")
        print(f"3. 指派结果: {assigned_pairs}")
        timeline_rows = result.get("table_output", {}).get("schedule_timeline", {}).get("data", [])
        print(
            "4. 准备开始时间 / 预计完成时间: "
            + "；".join(
                f"{row['airport_name']} 架次{row['sortie_index']}-{row['relay_leg']}/{row['relay_legs']} "
                f"{row['planned_start_time']} -> {row['planned_end_time']}"
                f"（准备开始前恢复等待{row['charge_wait_before_takeoff_min']}min）"
                for row in timeline_rows
            )
        )
        print(f"5. 航线 route_id: {route_ids}")
        print(
            "6. 单架次飞行/作业/总时长: "
            + "；".join(
                f"{row['target']} {row['flight_min']}min / {row['work_min']}min / {row['total_min']}min"
                for row in decision_chain
            )
        )
        print("7. 三套方案 + 推荐: 见下方方案列表")
        print("8. 决策依据: 见推荐理由与方案差异")
        print("9. 不可行原因 InfeasibleReason: 见未进入最终输出的策略 / 未分配航点")
        print("10. 抢占 / 接力 / 协同决策: 见规则覆盖与执行明细")
    else:
        print("1. 候选机场集合: -")
        print("2. 合规机场集合: -")
        print("3. 指派结果: -")
        print("4. 准备开始时间 / 预计完成时间: -")
        print("5. 航线 route_id: -")
        print("6. 单架次飞行/作业/总时长: -")
        print("7. 三套方案 + 推荐: 见下方方案列表")
        print("8. 决策依据: 见推荐理由与方案差异")
        print("9. 不可行原因 InfeasibleReason: 见未生成可执行方案说明")
        print("10. 抢占 / 接力 / 协同决策: 见规则覆盖与执行明细")
    print("")
    print("方案与执行结果:")
    if not result["schemes"]:
        print("=" * 72)
        print("未生成可执行方案。下面是三种策略的尝试结果，便于排查卡在哪个约束。")
        for scheme in result.get("attempted_schemes", []):
            s = scheme["summary"]
            print(
                f"- {scheme['scheme_label']} ({scheme['scheme_name']}): "
                f"状态={scheme['status']} | 对象={s['assigned_target_count']}/{wo['target_count']} | "
                f"分段={s['segment_count']} | 机场={s['used_airport_count']} | 架次={s['sortie_count']} | "
                f"最低余量={s['min_battery_remaining_pct']}% | 原因={scheme.get('reject_reason', '未通过筛选')}"
            )
    for scheme in result["schemes"]:
        s = scheme["summary"]
        marker = " <- 推荐" if scheme["scheme_name"] == rec["scheme_name"] else ""
        print("=" * 72)
        print(
            f"方案: {scheme['scheme_label']} ({scheme['scheme_name']}){marker}"
        )
        print(f"说明: {scheme['description']}")
        print(
            f"执行结果: 状态={scheme['status']} | "
            f"对象={s['assigned_target_count']}/{wo['target_count']} | "
            f"分段={s['segment_count']} | 机场={s['used_airport_count']} | 架次={s['sortie_count']} | "
            f"总距离={s['total_route_distance_m']}m | 最长耗时={s['max_plan_duration_min']}min | "
            f"整体完成={s.get('estimated_completion_min', 0)}min | "
            f"最低余量={s['min_battery_remaining_pct']}% | 风险段={s['warn_or_worse_count']}"
        )
        print(
            "结果摘要: "
            + "；".join(
                f"{row['target']}[{row['route_id']}]→{row['assigned_airport_name']} / {row['drone_id']} "
                f"({row['flight_min']}+{row['work_min']}+{PARAMS['P040_prepare_min']}+{PARAMS['P011_safety_fixed_min']}={row['total_min']}min)"
                for row in scheme.get("assignments", [])
            )
        )
        print("执行明细:")
        grouped: Dict[Tuple[Any, ...], List[Dict[str, Any]]] = {}
        for plan in scheme["plans"]:
            route_labels = []
            for r in plan["route"]:
                route_labels.append(
                    f"{r['obj_name']}[{r['first_waypoint']} -> {r['last_waypoint']}，{r['waypoint_count']}航点]"
                )
            objects = "；".join(route_labels) or "未知对象"
            key = (
                plan["airport_uid"],
                plan["sortie_index"],
                tuple((r["obj_id"], r["first_waypoint"], r["last_waypoint"]) for r in plan["route"]),
            )
            grouped.setdefault(key, []).append({**plan, "_objects": objects})
        for legs in grouped.values():
            first = legs[0]
            ok = all(p["feasible_after_endurance_check"] for p in legs)
            flag = "OK" if ok else "ENDURANCE_RISK"
            print(
                f"  - 巡检段: {first['_objects']} | {first['airport_name']} / "
                f"{first['drone_name']}({first['drone_id']})"
            )
            if first["relay_legs"] > 1:
                print(f"    接力原因: {first['relay_reason']}")
                print(
                    f"    接力计划: 共{first['relay_legs']}段；"
                    f"每段作业{first['work_min']}min，每段都重新从机场起降并保留安全余量；"
                    f"返航后按满充/换电{first['charging_duration_min']}min估算资源恢复。"
                )
                for leg in legs:
                    print(
                        f"      第{leg['relay_leg']}段: 准备{leg['prepare_min']}min + 飞行{leg['flight_min']}min "
                        f"+ 作业{leg['work_min']}min + 安全冗余{leg['safety_min']}min = {leg['total_min']}min；"
                        f"准备开始{leg.get('planned_start_time', '')} -> {leg.get('planned_end_time', '')}；"
                        f"准备开始前恢复等待{leg.get('charge_wait_before_takeoff_min', 0)}min；"
                        f"本段后恢复完成{leg.get('resource_recovery_end_time', '')}；"
                        f"预计耗电{leg['battery_use_pct']}%，剩余{leg['battery_remaining_pct']}%，"
                        f"状态={leg['battery_level']}"
                    )
            else:
                print("    接力: 不需要接力，单段可完成。")
                print(
                    f"    执行: 准备{first['prepare_min']}min + 飞行{first['flight_min']}min "
                    f"+ 作业{first['work_min']}min + 安全冗余{first['safety_min']}min = {first['total_min']}min；"
                    f"准备开始{first.get('planned_start_time', '')} -> {first.get('planned_end_time', '')}；"
                    f"准备开始前恢复等待{first.get('charge_wait_before_takeoff_min', 0)}min；"
                    f"本段后恢复完成{first.get('resource_recovery_end_time', '')}；"
                    f"预计耗电{first['battery_use_pct']}%，剩余{first['battery_remaining_pct']}%，"
                    f"状态={first['battery_level']}"
                )
            print(f"    结果: [{flag}]")
    filtered = [
        s for s in result.get("attempted_schemes", [])
        if s["scheme_name"] not in {x["scheme_name"] for x in result["schemes"]}
    ]
    if filtered and result["schemes"]:
        print("")
        print("未进入最终输出的策略:")
        for scheme in filtered:
            s = scheme["summary"]
            print(
                f"- {scheme['scheme_label']} ({scheme['scheme_name']}): "
                f"状态={scheme['status']} | 对象={s['assigned_target_count']}/{wo['target_count']} | "
                f"架次={s['sortie_count']} | 原因={scheme.get('reject_reason', '被筛选')}"
            )
            failed = [
                p for p in result.get("table_output", {}).get("infeasible_reason", {}).get("data", [])
                if p.get("scheme_name") == scheme["scheme_name"]
            ]
            if failed:
                for item in failed[0].get("failed_plans", []):
                    print(
                        f"    - 失败段: {item['airport_name']} / 无人机{item['drone_id']} / "
                        f"架次{item['sortie_index']} 第{item['relay_leg']}/{item['relay_legs']}段 / "
                        f"{item['segment_start_waypoint']} -> {item['segment_end_waypoint']} / "
                        f"总时长{item['total_min']}min > 上限{item['max_allowed_min']}min / "
                        f"剩余电量{item['battery_remaining_pct']}% / {item['reason']}"
                    )
    print("")
    print("规则覆盖:")
    for row in result["rule_audit"]:
        print(f"- {row['param']}: {row['json_fields']} -> {row['usage']}")
    if result["unassigned"]:
        print("")
        print("未分配航点:")
        for item in result["unassigned"][:10]:
            print(f"- {item['target']}")
        if len(result["unassigned"]) > 10:
            print(f"... 还有 {len(result['unassigned']) - 10} 个")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Path to 无人机调度结构.json")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Path to write result JSON")
    parser.add_argument("--no-output", action="store_true", help="Only print summary")
    parser.add_argument("--llm-explain", action="store_true", help="Rewrite explanation fields with the LLM")
    args = parser.parse_args()

    data = load_structure(args.input)
    result = solve(data)
    if args.llm_explain:
        result = enrich_result_with_llm(result)
    print_summary(result)

    if not args.no_output:
        out_path = os.path.abspath(args.output)
        out_dir = os.path.dirname(out_path) or os.getcwd()
        diagram_dir = os.path.join(out_dir, "core_demo_diagrams")
        render_scheme_diagrams(result, diagram_dir)
        os.makedirs(out_dir, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(result["table_output"], f, ensure_ascii=False, indent=2)
        print("")
        print(f"结果已写入: {out_path}")


if __name__ == "__main__":
    main()

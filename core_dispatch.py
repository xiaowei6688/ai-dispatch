from __future__ import annotations

import copy
import json
import math
import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any, Dict, Iterable, List, Optional, Tuple

from dispatch_config import CONFIGURABLE_PARAMS

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
    raw_route_points: List[Dict[str, Any]]

@dataclass
class WorkSegment:
    group_id: str
    group_name: str
    airport_uid: str
    route_points: List[Dict[str, Any]]
    raw_route_points: List[Dict[str, Any]]
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
        raw_route_points = [copy.deepcopy(raw) for raw in group.get("obj_data", [])]
        route_points = [
            {
                "index": raw.get("index", point_index),
                "lon": float(raw.get("longitude", raw.get("lon"))),
                "lat": float(raw.get("latitude", raw.get("lat"))),
                "altitude": float(raw.get("altitude", 0) or 0),
            }
            for point_index, raw in enumerate(raw_route_points)
        ]
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
                raw_route_points=raw_route_points,
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
    raw_route_points = list(getattr(item, "raw_route_points", []) or [])
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
                raw_route_points,
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
        raw_chunk = raw_route_points[start:end]
        if not chunk:
            continue
        chunks.append(
            WorkSegment(
                item.group_id,
                item.group_name,
                airport_uid,
                chunk,
                raw_chunk,
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
    raw_route_points = list(getattr(item, "raw_route_points", []) or [])
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
                raw_route_points[start:end],
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
            raw_route_points,
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
            raw_route_points[start:end],
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

    buckets: Dict[str, List[Tuple[Dict[str, Any], Dict[str, Any]]]] = {}
    for point_index, raw in enumerate(point.route_points):
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
        original_raw = point.raw_route_points[point_index] if point_index < len(point.raw_route_points) else raw
        buckets.setdefault(chosen.uid, []).append((raw, original_raw))

    segments = [
        WorkSegment(
            point.group_id,
            point.group_name,
            airport_uid,
            [route_point for route_point, _ in route_points],
            [copy.deepcopy(raw_point) for _, raw_point in route_points],
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
                start_wp = leg_items[0].route_points[0].get("index") if leg_items[0].route_points else ""
                end_wp = leg_items[-1].route_points[-1].get("index") if leg_items[-1].route_points else ""
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
                                "first_waypoint": item.route_points[0].get("index") if item.route_points else "",
                                "last_waypoint": item.route_points[-1].get("index") if item.route_points else "",
                                "waypoints": item.route_points,
                                "raw_waypoints": [copy.deepcopy(point) for point in item.raw_route_points],
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
    }

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

def build_track_list_output(result: Dict[str, Any]) -> Dict[str, Any]:
    """把推荐方案折叠成下游平台需要的航线列表结构。

    trackList 的每一项对应推荐方案中的一条已规划航线
    （即 ``plans[].route[]``，同一作业对象被拆分到多个机场/接力段时会产生多条）。
    ``trackContent`` 为对象结构：保留输入 ``obj_data`` 中每个航点的原始字段，
    并统计航点数量写入 ``datas[0].wy_count``。
    """
    scheme = result.get("recommended_scheme") or {}
    track_list = []
    for plan in scheme.get("plans", []):
        for route in plan.get("route", []):
            raw_waypoints = route.get("raw_waypoints") or route.get("waypoints") or []
            items = [copy.deepcopy(point) for point in raw_waypoints]
            track_content = {
                "datas": [
                    {
                        "deviceType": 0,
                        "wy_count": len(items),
                        "items": items,
                    }
                ],
                "manufacturer_name": "众芯汉创",
                "version": "1.3",
            }
            track_list.append(
                {
                    "trackId": str(uuid.uuid4()),
                    "trackPath": "",
                    "trackContent": track_content,
                    "trackType": "json",
                    "airportGuid": plan.get("airport_uid"),
                    "airportName": plan.get("airport_name"),
                    "objId": route.get("obj_id"),
                }
            )
    return {
        "workOrderGuid": (result.get("work_order") or {}).get("guid"),
        "trackList": track_list,
    }

_SCHEME_METRIC_KEYS = (
    "assigned_target_count",
    "unassigned_target_count",
    "used_airport_count",
    "segment_count",
    "sortie_count",
    "total_route_distance_m",
    "max_plan_duration_min",
    "min_battery_remaining_pct",
    "avg_battery_remaining_pct",
    "warn_or_worse_count",
    "return_risk_count",
    "estimated_completion_min",
)

def _scheme_metrics(scheme: Dict[str, Any]) -> Dict[str, Any]:
    """从方案 summary 中抽取少量、确定性的指标，供说明生成与结构化返回复用。"""
    s = scheme.get("summary") or {}
    metrics = {key: s.get(key, 0) for key in _SCHEME_METRIC_KEYS}
    metrics["within_work_order_window"] = bool(s.get("within_work_order_window", True))
    return metrics

def _scheme_flight_details(scheme: Dict[str, Any]) -> Dict[str, Any]:
    """把方案 plans 压缩成航段叙述事实，供说明生成使用。"""
    segments = []
    for plan in scheme.get("plans") or []:
        objects = list(plan.get("object_names") or [])
        route_objects = [
            r.get("obj_name") for r in plan.get("route") or [] if r.get("obj_name")
        ]
        if not objects and route_objects:
            objects = route_objects
        battery_level = plan.get("battery_level") or "正常"
        relay_legs = plan.get("relay_legs") or 1
        needs_charge_relay = bool(
            plan.get("relay_reason")
            or relay_legs != 1
            or battery_level in ("接力", "返航")
        )
        segments.append(
            {
                "sortie_index": plan.get("sortie_index"),
                "relay_leg": plan.get("relay_leg"),
                "relay_legs": relay_legs,
                "airport_name": plan.get("airport_name"),
                "drone_id": plan.get("drone_id"),
                "objects": objects,
                "waypoint_count": plan.get("point_count"),
                "start_waypoint": plan.get("segment_start_waypoint"),
                "end_waypoint": plan.get("segment_end_waypoint"),
                "flight_min": plan.get("flight_min"),
                "work_min": plan.get("work_min"),
                "total_min": plan.get("total_min"),
                "battery_level": battery_level,
                "battery_remaining_pct": plan.get("battery_remaining_pct"),
                "needs_charge_relay": needs_charge_relay,
                "relay_reason": plan.get("relay_reason") or "",
            }
        )
    return {
        "segment_count": len(segments),
        "charge_relay_count": sum(1 for s in segments if s["needs_charge_relay"]),
        "segments": segments,
    }

def _scheme_status(scheme: Dict[str, Any]) -> str:
    if scheme.get("reject_reason"):
        return "rejected"
    return scheme.get("status") or "unknown"

def _scheme_deterministic_description(scheme: Dict[str, Any]) -> str:
    """无大模型时的兜底方案说明，基于结构化指标与航段细节生成。"""
    label = scheme.get("scheme_label") or scheme.get("scheme_name") or ""
    base = scheme.get("description") or ""
    m = _scheme_metrics(scheme)
    parts = [f"{label}：{base}".strip(" ：")]
    parts.append(
        f"共分配{m['assigned_target_count']}个作业对象、{m['segment_count']}个任务段，"
        f"使用{m['used_airport_count']}个机场，拆分为{m['sortie_count']}个架次；"
    )
    parts.append(
        f"总航线距离{m['total_route_distance_m']}米，单段最长耗时{m['max_plan_duration_min']}分钟，"
        f"预计整体完成约{m['estimated_completion_min']}分钟；"
    )
    parts.append(
        f"电量方面最低剩余{m['min_battery_remaining_pct']}%、平均剩余{m['avg_battery_remaining_pct']}%，"
        f"预警及以下段数{m['warn_or_worse_count']}、返航风险段数{m['return_risk_count']}。"
    )
    details = _scheme_flight_details(scheme)
    segments = details["segments"]
    if segments:
        narrations = []
        for i, seg in enumerate(segments, start=1):
            objects = "、".join(seg["objects"]) if seg["objects"] else "无"
            if seg["needs_charge_relay"]:
                charge_note = f"，飞完剩余电量{seg['battery_remaining_pct']}%（{seg['battery_level']}），需要返航充电/接力"
            else:
                charge_note = f"，飞完剩余电量{seg['battery_remaining_pct']}%（{seg['battery_level']}），无需返航充电"
            relay_note = ""
            if seg["relay_legs"] and seg["relay_legs"] > 1:
                relay_note = f"，该航点被拆为{seg['relay_legs']}段接力（当前为第{seg['relay_leg']}段）"
            narrations.append(
                f"第{i}个航段：机场{seg['airport_name']}的{seg['drone_id']}执行，"
                f"巡检对象为{objects}，共{seg['waypoint_count']}个航点，"
                f"飞行约{seg['flight_min']}分钟+作业约{seg['work_min']}分钟，总计约{seg['total_min']}分钟"
                f"{charge_note}{relay_note}"
            )
        parts.append("飞行细节：" + "；".join(narrations) + "。")
    if scheme.get("reject_reason"):
        parts.append(f"该方案未入选，原因：{scheme['reject_reason']}")
    return "".join(parts)

def _llm_describe_schemes(client: Any, payloads: List[Dict[str, Any]]) -> Dict[str, str]:
    """一次性让大模型为多个方案生成详细业务说明，返回 {scheme_name: description}。"""
    prompt = (
        "你是无人机巡检调度结果解释器，服务于“众芯汉创”平台。"
        "下面 JSON 数组中的每个元素代表一个调度方案，请为每个方案生成一段面向业务人员的中文说明，"
        "尽量详细，讲清楚该方案是什么、关键指标表现、它与其他方案定位上的差异，并把 metrics 里的关键数字自然融入说明；"
        "还要口述飞行细节：共有几个航段、每个航段由哪个机场的哪架无人机执行、各自巡检哪些对象，"
        "飞完后哪些需要返航充电或接力；"
        "如果标记 recommended=true 要说明它为何更值得推荐；如果有 reject_reason 要说明它为什么被筛除。\n"
        "硬性要求：只能基于给定的事实和数字，不得编造任何新的数字、机场名、无人机ID或状态；"
        "输出必须是严格 JSON 数组，每个元素只包含 scheme_name 和 description 两个字段，"
        "不要输出任何解释或 Markdown 代码块。\n"
        f"数据：{json.dumps(payloads, ensure_ascii=False)}"
    )
    try:
        from langchain_core.messages import HumanMessage

        resp = client.invoke([HumanMessage(content=prompt)])
        raw = _extract_llm_output(resp)
    except Exception:
        return {}
    parsed = None
    try:
        parsed = json.loads(raw)
    except Exception:
        start = raw.find("[")
        end = raw.rfind("]")
        if start != -1 and end != -1 and start <= end:
            try:
                parsed = json.loads(raw[start:end + 1])
            except Exception:
                parsed = None
    if not isinstance(parsed, list):
        return {}
    out: Dict[str, str] = {}
    for row in parsed:
        if not isinstance(row, dict):
            continue
        key = row.get("scheme_name")
        value = row.get("description")
        if isinstance(key, str) and isinstance(value, str) and value.strip():
            out[key] = value.strip()
    return out

def build_scheme_explanations(result: Dict[str, Any]) -> List[Dict[str, Any]]:
    """为接口返回生成不同方案的说明列表（含结构化指标）。"""
    attempted = result.get("attempted_schemes") or []
    if not attempted:
        generated = result.get("schemes") or []
        attempted = generated
    recommended = result.get("recommended_scheme") or {}
    recommended_name = recommended.get("scheme_name")

    entries = []
    payloads = []
    for scheme in attempted:
        if not isinstance(scheme, dict):
            continue
        status = _scheme_status(scheme)
        is_recommended = bool(recommended_name and scheme.get("scheme_name") == recommended_name)
        metrics = _scheme_metrics(scheme)
        flight_details = _scheme_flight_details(scheme)
        payload = {
            "scheme_name": scheme.get("scheme_name"),
            "scheme_label": scheme.get("scheme_label"),
            "recommended": is_recommended,
            "status": status,
            "base_description": scheme.get("description") or "",
            "reject_reason": scheme.get("reject_reason") or "",
            "metrics": metrics,
            "flight_details": flight_details,
        }
        entries.append((scheme, payload))

        flat = dict(payload)
        flat["metrics"] = metrics
        flat["flight_details"] = flight_details
        payloads.append(flat)

    client = _load_llm_client()
    llm_descriptions: Dict[str, str] = {}
    if client and payloads:
        llm_descriptions = _llm_describe_schemes(client, payloads)

    output = []
    for scheme, payload in entries:
        metrics = payload["metrics"]
        description = llm_descriptions.get(payload["scheme_name"]) or _scheme_deterministic_description(scheme)
        item = {
            "schemeName": payload["scheme_name"],
            "schemeLabel": payload["scheme_label"],
            "recommended": payload["recommended"],
            "status": payload["status"],
            "description": description,
        }
        if payload.get("reject_reason"):
            item["rejectReason"] = payload["reject_reason"]
        output.append(item)
    return output

def solve(data: Dict[str, Any], work_sec_per_waypoint: Optional[int] = None) -> Dict[str, Any]:
    PARAMS.clear()
    PARAMS.update(CONFIGURABLE_PARAMS)
    apply_runtime_params(data)
    if work_sec_per_waypoint is not None:
        PARAMS["P039_work_sec_per_waypoint"] = work_sec_per_waypoint
    work_order = data.get("work_order", {})
    schemes, attempted_schemes = generate_schemes(data)
    if not schemes:
        return {
            "work_order": build_work_order_summary(data),
            "recommended_scheme": None,
            "schemes": [],
            "attempted_schemes": attempted_schemes,
        }
    recommended = min(schemes, key=lambda s: recommendation_key(s, work_order))
    return {
        "work_order": recommended["work_order"],
        "recommended_scheme": recommended,
        "schemes": schemes,
        "attempted_schemes": attempted_schemes,
    }

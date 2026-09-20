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
    "P039_work_sec_per_waypoint": 10,
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



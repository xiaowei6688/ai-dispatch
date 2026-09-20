from __future__ import annotations

from typing import Any, Dict, Optional

from fastapi import FastAPI, HTTPException
import uvicorn

from core_dispatch import build_scheme_explanations, build_track_list_output, solve


app = FastAPI(title="ai-dispatch", version="0.1.0")


@app.get("/")
def health() -> Dict[str, str]:
    return {"status": "ok"}


@app.post("/dispatch")
def dispatch(
    payload: Dict[str, Any],
    work_sec_per_waypoint: Optional[int] = None,
) -> Dict[str, Any]:
    """调度接口。

    ``work_sec_per_waypoint`` 为单个航点作业时长（秒）：
    不传时使用 dispatch_config.py 中的默认值 10 秒；
    真实巡检杆塔等更耗时场景可通过该参数传递，例如 5 分钟传 300。
    """
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="Request body must be a JSON object.")
    if work_sec_per_waypoint is not None and work_sec_per_waypoint <= 0:
        raise HTTPException(
            status_code=400,
            detail="work_sec_per_waypoint must be a positive integer.",
        )
    result = solve(payload, work_sec_per_waypoint=work_sec_per_waypoint)
    response = build_track_list_output(result)
    response["schemes"] = build_scheme_explanations(result)
    return response


def main() -> None:
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False)


if __name__ == "__main__":
    main()

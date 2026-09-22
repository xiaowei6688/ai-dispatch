from __future__ import annotations

import threading
from typing import Any, Dict, Optional

import httpx
import uvicorn
from fastapi import BackgroundTasks, FastAPI, HTTPException

from core_dispatch import build_scheme_explanations, solve


app = FastAPI(title="ai-dispatch", version="0.1.0")

_COMPUTE_LOCK = threading.Lock()


@app.get("/")
def health() -> Dict[str, str]:
    return {"status": "ok"}


def _validate(payload: Any, work_sec_per_waypoint: Optional[int]) -> None:
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="Request body must be a JSON object.")
    if work_sec_per_waypoint is not None and work_sec_per_waypoint <= 0:
        raise HTTPException(
            status_code=400,
            detail="work_sec_per_waypoint must be a positive integer.",
        )


def build_response(result: Dict[str, Any]) -> Dict[str, Any]:
    schemes = build_scheme_explanations(result)
    recommended_track_list = next(
        (scheme.get("trackList") or [] for scheme in schemes if scheme.get("recommended")),
        [],
    )
    return {
        "workOrderGuid": (result.get("work_order") or {}).get("guid"),
        "trackList": recommended_track_list,
        "schemes": schemes,
    }


def _compute_response(data: Dict[str, Any], work_sec_per_waypoint: Optional[int]) -> Dict[str, Any]:
    with _COMPUTE_LOCK:
        result = solve(data, work_sec_per_waypoint=work_sec_per_waypoint)
        return build_response(result)


def _post_callback(url: str, body: Dict[str, Any]) -> None:
    httpx.post(url, json=body, timeout=30.0)


def _run_dispatch_task(
    data: Dict[str, Any],
    work_sec_per_waypoint: Optional[int],
) -> None:
    callback_url = data.get("callback_url", "")
    try:
        print(f"开始调度: {data["work_order"]["woker_order_guid"]}")
        response = _compute_response(data, work_sec_per_waypoint)
        print("执行完成...")
    except Exception as exc:  # noqa: BLE001
        if callback_url:
            try:
                _post_callback(callback_url, {"code":200, "msg": "failed", "error": str(exc)})
            except Exception as e:
                print(e.__str__())
                pass
        return

    if callback_url:
        try:
            _post_callback(callback_url, {"code":200, "msg": "success", **response})
            print("发送完成...")
        except Exception as e:
            print(e.__str__())
            pass


@app.post("/dispatch")
def dispatch(
    payload: Dict[str, Any],
    work_sec_per_waypoint: Optional[int] = None,
) -> Dict[str, Any]:
    _validate(payload, work_sec_per_waypoint)
    return _compute_response(payload, work_sec_per_waypoint)


@app.post("/dispatch/async")
def dispatch_async(
    payload: Dict[str, Any],
    background_tasks: BackgroundTasks,
    work_sec_per_waypoint: Optional[int] = None,
) -> Dict[str, Any]:
    _validate(payload, work_sec_per_waypoint)
    background_tasks.add_task(
        _run_dispatch_task,
        payload,
        work_sec_per_waypoint,
    )
    return {"code":200, "msg": "success"}


def main() -> None:
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False)


if __name__ == "__main__":
    main()

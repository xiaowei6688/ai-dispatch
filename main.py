from __future__ import annotations

from typing import Any, Dict

from fastapi import FastAPI, HTTPException
import uvicorn

from core_dispatch_demo import enrich_result_with_llm, solve


app = FastAPI(title="ai-dispatch", version="0.1.0")


@app.get("/")
def health() -> Dict[str, str]:
    return {"status": "ok"}


@app.post("/dispatch")
def dispatch(payload: Dict[str, Any], llm_explain: bool = True) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="Request body must be a JSON object.")
    result = solve(payload)
    if llm_explain:
        result = enrich_result_with_llm(result)
    return result["table_output"]


def main() -> None:
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False)


if __name__ == "__main__":
    main()

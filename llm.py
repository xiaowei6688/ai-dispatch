from __future__ import annotations

import os
from pathlib import Path

from langchain_openai import ChatOpenAI


def _load_dotenv(path: Path) -> None:
    if not path.exists():
        return
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))

_load_dotenv(Path(__file__).with_name(".env"))

llm_client = ChatOpenAI(
    model=os.getenv("LLM_MODEL", "qwen3.8-27b"),
    api_key=os.getenv("LLM_API_KEY"),
    base_url=os.getenv("LLM_BASE_URL"),
    temperature=float(os.getenv("LLM_TEMPERATURE", "0.2")),
    streaming=os.getenv("LLM_STREAMING", "true").lower() in {"1", "true", "yes", "y"},
    stream_usage=os.getenv("LLM_STREAM_USAGE", "true").lower() in {"1", "true", "yes", "y"},
    extra_body={"enable_thinking": os.getenv("LLM_ENABLE_THINKING", "false").lower() in {"1", "true", "yes", "y"}},
)

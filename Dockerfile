FROM swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/python:3.11.15-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY core_dispatch.py dispatch_config.py llm.py main.py ./

RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir \
        "fastapi>=0.115.0" \
        "httpx>=0.27.0" \
        "langchain-openai>=1.1.0" \
        "uvicorn>=0.30.0"

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]

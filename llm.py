from langchain_openai import ChatOpenAI

llm_client = ChatOpenAI(
    model="qwen3.8-27b",
    api_key="sk-ws-H.EHHYIHX.H07h.MEUCIQDxHya1iyGWMhtVpjAwSAnthHwG5JUtbQ83vT8lMIObiQIgRzhxlSh8hUhXuUXGrF7d0gLJt8eBHTAjByQZk0Di",
    base_url="https://ws-uivspyfugj0brmmb.cn-beijing.maas.aliyuncs.com/compatible-mode/v1",
    temperature=0.2,
    streaming=True,
    stream_usage=True,
    extra_body={'enable_thinking': False}
)

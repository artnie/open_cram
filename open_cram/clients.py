import os
import httpx
from openai import OpenAI

class OpenWebUIClient:
    def __init__(self, base_url: str, timeout_s: float = 30.0, api_key: str | None = None):
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout_s
        self.api_key = api_key or os.getenv('OPENWEBUI_API_KEY')

    def chat(self, prompt: str) -> str:
        # Example REST call; adapt endpoint/payload to your Open-WebUI setup
        headers = {'Authorization': f'Bearer {self.api_key}'} if self.api_key else {}
        payload = {"inputs": prompt}
        with httpx.Client(timeout=self.timeout) as c:
            r = c.post(f'{self.base_url}/api/chat', json=payload, headers=headers)
            r.raise_for_status()
            data = r.json()
        return data.get('text') or data

class OpenAIClient:
    def __init__(self, model: str, timeout_s: float = 30.0):

        self.client = OpenAI(timeout=timeout_s)  # uses OPENAI_API_KEY env var
        self.model = model

    def chat(self, prompt: str) -> str:
        resp = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role":"user","content":prompt}]
        )
        return resp.choices[0].message.content
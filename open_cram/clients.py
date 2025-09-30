import os
import httpx
from openai import OpenAI
import requests
import json

class OpenWebUIClient:
    def __init__(self, base_url: str = 'http://192.168.200.10:3000', timeout_s: float = 30.0, api_key: str = None):
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout_s
        self.api_key = api_key or os.getenv('OPENWEBUI_API_KEY')

    def chat(self, prompt: str, model: str ="deepseek-r1:32b") -> str:
        # Example REST call; adapt endpoint/payload to your Open-WebUI setup
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json'
        }
        payload = {
            "model": model,
            "messages":
                [
                    {
                        "role": "user",
                        "content": prompt
                    }
                ]
        }
        # json_data = json.dumps(payload)
        # response = requests.post(f'{self.base_url}/api/chat/completions', headers=headers, data=json_data)
        # result = response.json()
        # return str(result) + str(payload)
        with httpx.Client(timeout=self.timeout) as c:
            r = c.post(f'{self.base_url}/api/chat/completions', json=payload, headers=headers)
            r.raise_for_status()
            data = r.json()
        return data.get('text') or data


# def get_response(input_string):
#     output_string = re.sub(r'<think>.*?</think>', '', input_string, flags=re.DOTALL)
#     return output_string

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
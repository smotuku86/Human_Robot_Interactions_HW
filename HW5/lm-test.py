from openai import OpenAI
from HW5.secret_key import sk


# Modify OpenAI's API key and API base to use the server.
openai_api_key = sk
openai_api_base = "https://llm-api.arc.vt.edu/api/v1"

client = OpenAI(
    api_key=openai_api_key,
    base_url=openai_api_base,
)

messages = [
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "What is Virginia Tech's mascot?"},
]

model = "gpt-oss-120b"
response = client.chat.completions.create(model=model, messages=messages)
code_str = response.choices[0].message.content
print(code_str)

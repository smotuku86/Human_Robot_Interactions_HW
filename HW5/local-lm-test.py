from openai import OpenAI

# Ollama runs locally and exposes an OpenAI-compatible API
client = OpenAI(
    api_key="ollama",  # required but ignored
    base_url="http://127.0.0.1:11434/v1"
)

messages = [
    {"role": "system", "content": "You are a helpful assistant and write clean code."},
    {"role": "user", "content": "What is Virginia Tech's mascot?"}
]

model = "qwen2.5-coder:7b"

response = client.chat.completions.create(
    model=model,
    messages=messages
)

code_str = response.choices[0].message.content
print(code_str)

#ask user for request to give llm a task
task = input("What task would you like the language model to perform? ")
messages.append({"role": "user", "content": task})
response = client.chat.completions.create(
    model=model,
    messages=messages
)

code_str = response.choices[0].message.content
print(code_str)
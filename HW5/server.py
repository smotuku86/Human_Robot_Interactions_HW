from fastapi import FastAPI
from pydantic import BaseModel
import queue
import threading
from fastapi.responses import HTMLResponse
import time

# --------- Task queue ---------
task_queue = queue.Queue()

# --------- FastAPI setup ---------
app = FastAPI()

# --------- Task handler thread ---------
class TaskRequest(BaseModel):
    task: str

# --------- HTML page ---------
@app.get("/", response_class=HTMLResponse)
def home():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Teleop</title>
    </head>
    <body>
        <h2>Robot Teleop Console</h2>
        <input type="text" id="taskInput" placeholder="Enter a task" style="width:300px;">
        <button onclick="sendTask()">Send</button>
        <p id="status"></p>

        <script>
            async function sendTask() {
                const task = document.getElementById("taskInput").value;
                const response = await fetch("/task", {
                    method: "POST",
                    headers: {"Content-Type": "application/json"},
                    body: JSON.stringify({task: task})
                });
                const data = await response.json();
                document.getElementById("status").innerText = "Sent: " + data.task;
                document.getElementById("taskInput").value = "";
            }
        </script>
    </body>
    </html>
    """

@app.post("/task")
def add_task(req: TaskRequest):
    """
    Receive a task from the user (POST /task {"task": "pick up cube1"})
    """
    task_queue.put(req.task)
    return {"status": "task received", "task": req.task}

@app.get("/next_task")
def next_task():
    if not task_queue.empty():
        return {"task": task_queue.get()}
    return {"task": None}

# --------- Background thread to run FastAPI ---------
def start_api():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

threading.Thread(target=start_api, daemon=True).start()

# ---------- Main loop: print tasks from queue ----------
print("Server running on http://localhost:8000")
while True:
    if not task_queue.empty():
        task = task_queue.get()
        print("Got task:", task)
    time.sleep(0.1)  # avoid busy waiting

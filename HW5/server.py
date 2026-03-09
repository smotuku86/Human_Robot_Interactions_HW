from fastapi import FastAPI
from pydantic import BaseModel
import queue
import threading
from fastapi.responses import HTMLResponse, StreamingResponse
import time
import json

# --------- Queues ---------
task_queue = queue.Queue()
llm_messages = queue.Queue()
score_messages = queue.Queue()
time_messages = queue.Queue()

# --------- FastAPI setup ---------
app = FastAPI()

# --------- Request Models ---------
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

        <div style="display:flex; gap:16px; margin: 12px 0;">
            <div style="flex:1; background:#f8f9fa; border:1px solid #dee2e6; border-radius:6px; padding:12px;">
                <div style="font-size:11px; font-weight:600; color:#495057; text-transform:uppercase; letter-spacing:0.05em;">Score</div>
                <div id="score" style="font-size:28px; font-weight:700; color:#212529; font-family:'Segoe UI',sans-serif;">—</div>
            </div>
            <div style="flex:1; background:#f8f9fa; border:1px solid #dee2e6; border-radius:6px; padding:12px;">
                <div style="font-size:11px; font-weight:600; color:#495057; text-transform:uppercase; letter-spacing:0.05em;">Time Elapsed</div>
                <div id="time" style="font-size:28px; font-weight:700; color:#212529; font-family:'Segoe UI',sans-serif;">—</div>
            </div>
        </div>

        <h3>LLM Status</h3>
        <div id="status" style="
            font-family: 'Segoe UI', sans-serif;
            background: #f8f9fa;
            border: 1px solid #dee2e6;
            border-radius: 6px;
            padding: 12px;
            color: #888;
            font-style: italic;
        ">Waiting...</div>

        <style>
            .llm-table {
                border-collapse: collapse;
                width: 100%;
                font-family: 'Segoe UI', sans-serif;
                font-size: 14px;
            }
            .llm-table tr {
                border-bottom: 1px solid #e9ecef;
            }
            .llm-table tr:last-child {
                border-bottom: none;
            }
            .llm-table td.key {
                padding: 8px 16px 8px 8px;
                font-weight: 600;
                color: #495057;
                white-space: nowrap;
                vertical-align: top;
                width: 1%;
                text-transform: uppercase;
                font-size: 11px;
                letter-spacing: 0.05em;
            }
            .llm-table td.val {
                padding: 8px;
                color: #212529;
                word-break: break-word;
            }
            .llm-table td.val pre {
                margin: 0;
                background: #e9ecef;
                padding: 6px 8px;
                border-radius: 4px;
                font-size: 12px;
                white-space: pre-wrap;
            }
        </style>

        <script>

        async function sendTask() {
            const task = document.getElementById("taskInput").value;

            const response = await fetch("/task", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({task: task})
            });

            const data = await response.json();
            document.getElementById("status").innerHTML =
                `<span style="color:#888; font-style:italic; font-family:sans-serif;">Task sent: <strong>${data.task}</strong> — waiting for LLM response...</span>`;
            document.getElementById("taskInput").value = "";
        }

        const source = new EventSource("/stream");

        source.onmessage = function(event) {
            // Ignore keepalive pings
            if (event.data === "ping") return;
            console.log("LLM message:", event.data);

            const container = document.getElementById("status");
            container.style.cssText = "font-family:'Segoe UI',sans-serif; background:#f8f9fa; border:1px solid #dee2e6; border-radius:6px; padding:12px;";

            try {
                const parsed = JSON.parse(event.data);
                let html = '<table class="llm-table">';
                for (const [key, value] of Object.entries(parsed)) {
                    const displayVal = typeof value === "object"
                        ? `<pre>${JSON.stringify(value, null, 2)}</pre>`
                        : value;
                    html += `<tr><td class="key">${key}</td><td class="val">${displayVal}</td></tr>`;
                }
                html += '</table>';
                container.innerHTML = html;
            } catch (e) {
                // Fallback: plain text if not JSON
                container.textContent = event.data;
            }
        };

        const scoreSource = new EventSource("/stream_score");
        scoreSource.onmessage = function(event) {
            if (event.data === "ping") return;
            document.getElementById("score").textContent = event.data;
        };

        const timeSource = new EventSource("/stream_time");
        timeSource.onmessage = function(event) {
            if (event.data === "ping") return;
            document.getElementById("time").textContent = event.data + "s";
        };

        source.onerror = function(e) {
            console.warn("SSE connection error, reconnecting...", e);
        };

        </script>
    </body>
    </html>
    """


# --------- Receive user tasks ---------
@app.post("/task")
def add_task(req: TaskRequest):
    print("Received task:", req.task)
    task_queue.put(req.task)
    return {"status": "task received", "task": req.task}


# --------- Robot polls this ---------
@app.get("/next_task")
def next_task():
    if not task_queue.empty():
        task = task_queue.get()
        print("Robot fetched task:", task)
        return {"task": task}
    return {"task": None}


# --------- Receive robot / LLM response ---------
@app.post("/llm_response")
def receive_llm_response(data: dict):

    # Convert JSON → single line string (SSE safe)
    message = json.dumps(data)

    print("LLM PLAN RECEIVED:", message)

    llm_messages.put(message)

    return {"status": "received"}


# --------- Receive score ---------
@app.post("/update_score")
def update_score(data: dict):
    score_messages.put(str(data["score"]))
    return {"status": "received"}


# --------- Receive time ---------
@app.post("/update_time")
def update_time(data: dict):
    time_messages.put(str(data["time"]))
    return {"status": "received"}


# --------- Stream score to browser ---------
@app.get("/stream_score")
def stream_score():
    def event_stream():
        while True:
            try:
                message = score_messages.get(timeout=15)
                yield f"data: {message}\n\n"
            except queue.Empty:
                yield "data: ping\n\n"
    return StreamingResponse(event_stream(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


# --------- Stream time to browser ---------
@app.get("/stream_time")
def stream_time():
    def event_stream():
        while True:
            try:
                message = time_messages.get(timeout=15)
                yield f"data: {message}\n\n"
            except queue.Empty:
                yield "data: ping\n\n"
    return StreamingResponse(event_stream(), media_type="text/event-stream",
                             headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


# --------- Stream messages to browser ---------
@app.get("/stream")
def stream():

    def event_stream():
        while True:
            try:
                # Wait up to 15 seconds for a message
                message = llm_messages.get(timeout=15)
                yield f"data: {message}\n\n"
            except queue.Empty:
                # Send a keepalive ping so the browser doesn't time out
                yield "data: ping\n\n"

    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",  # Disables Nginx buffering if behind a proxy
        }
    )


# --------- Start API in background ---------
def start_api():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)


threading.Thread(target=start_api, daemon=True).start()

print("Server running on http://localhost:8000")

while True:
    time.sleep(1)
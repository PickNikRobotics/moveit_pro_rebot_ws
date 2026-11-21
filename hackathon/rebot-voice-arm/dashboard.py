"""Robot dashboard + voice agent: open http://localhost:8765 in Chrome.

Live camera, pick and dance buttons, emergency stop, status, and a voice agent.
Talk to it (hold the mic button, or turn on hands-free) and say things like
"pick up item 2", "do the wave", or "grab the tape, then dance". Actions queue up
and run one after another; "stop" cancels everything.

Short commands go straight to the arm. Anything longer or chattier goes to Claude
(ANTHROPIC_API_KEY in .env), which can call the arm's tools: pick_up_item, do_move,
stop_arm. Without a key, a small built-in responder handles it.

Dances/moves: record one with  ./teleop.sh move NAME  and it appears here right away.

  ./dashboard.sh            then open http://localhost:8765
  PORT=8766 ./dashboard.sh  run a second copy on another port
"""
import base64, glob, json, os, random, re, subprocess, threading, time
from collections import deque

import cv2
from flask import Flask, Response, jsonify, request, send_file

import plate_check
import replay_pick as rp
from voice_pickup import ITEMS, VOICE, api_key, which_item

HERE = os.path.dirname(os.path.abspath(__file__))
CHOSEN = os.path.join(rp.DATA, "chosen.json")
MOVES_DIR = os.path.join(rp.DATA, "moves")
SPEED = float(os.environ.get("PICK_SPEED", "0.5"))
DANCE_SPEED = float(os.environ.get("DANCE_SPEED", "1.0"))   # 1.0 = as recorded
PORT = int(os.environ.get("PORT", "8765"))
MAX_QUEUE = 6
app = Flask(__name__)


def env_value(name):
    if os.environ.get(name):
        return os.environ[name]
    try:
        for line in open(os.path.join(HERE, ".env")):
            if line.startswith(name + "="):
                return line.split("=", 1)[1].strip().strip('"')
    except FileNotFoundError:
        pass
    return None


def list_moves():
    """Recorded moves, by name. Rescanned every call so new recordings show up live."""
    found = glob.glob(os.path.join(MOVES_DIR, "*", "data", "chunk-*", "*.parquet"))
    names = {p.split(os.sep)[-4] for p in found}
    return sorted(n for n in names if not n.endswith(".new"))   # NAME.new = recording in progress


def spoken(name):
    return name.replace("_", " ")


# ---------- camera: one owner, shared with the replay's before/after pictures ----------

class Camera:
    def __init__(self):
        self.frame, self.lock = None, threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            c = cv2.VideoCapture(rp.CAM, cv2.CAP_V4L2)
            c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            c.set(3, 640); c.set(4, 480)
            while c.isOpened():
                ok, f = c.read()
                if not ok:
                    break
                with self.lock:
                    self.frame = f
            c.release()
            with self.lock:
                self.frame = None
            time.sleep(1.0)   # camera unplugged or busy: retry

    def get(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


cam = Camera()


def snap(path):
    time.sleep(0.3)   # let the stream catch up with the arm
    f = cam.get()
    if f is not None:
        cv2.imwrite(path, f)
    return f is not None


rp.snap = snap   # replay takes its pictures from our stream instead of opening the camera


# ---------- robot jobs: a queue of ("pick", item) / ("move", name), run one at a time ----------

state = {"busy": False, "job": None, "job_kind": None, "queue": [], "last": None, "last_action": None,
         "log": deque(maxlen=80), "audio": None, "audio_id": 0}
jobs = deque()
jobs_cv = threading.Condition()


def log(msg):
    state["log"].appendleft(f"{time.strftime('%H:%M:%S')}  {msg}")
    print(msg, flush=True)


def announce(text):
    """Speak this in the browser on its next status poll."""
    state["audio"] = tts_b64(text)
    state["audio_id"] += 1
    state["announce"] = text


def label(action):
    kind, arg = action
    return f"item {arg} ({ITEMS.get(arg, '?')})" if kind == "pick" else f"the {spoken(arg)}"


def enqueue(actions):
    """Add actions to the queue. Returns (accepted, rejected-with-reason)."""
    ok, bad = [], []
    chosen, moves = json.load(open(CHOSEN)), list_moves()
    with jobs_cv:
        for a in actions:
            kind, arg = a
            if kind == "pick" and str(arg) not in chosen:
                bad.append((a, f"no saved pick for item {arg}"))
            elif kind == "move" and arg not in moves:
                bad.append((a, f"no move called {spoken(arg)}"))
            elif len(jobs) >= MAX_QUEUE:
                bad.append((a, "the queue is full"))
            else:
                jobs.append(a); ok.append(a)
        state["queue"] = [label(j) for j in jobs]
        if ok:
            state["busy"] = True
        jobs_cv.notify()
    for a in ok:
        log(f"Queued {label(a)}")
    return ok, bad


def stop_all():
    with jobs_cv:
        jobs.clear()
        state["queue"] = []
    rp.STOP.set()
    log("STOP: arm stopping, queue cleared")


def run_pick(item):
    chosen = json.load(open(CHOSEN))
    log(f"Picking up item {item} ({ITEMS.get(item, '?')})")
    before, after, err = rp.replay(*rp.load_dir(os.path.join(rp.DATA, chosen[str(item)])),
                                   SPEED, f"dash_item{item}")
    if rp.STOP.is_set():
        state["last"] = {"item": item, "ok": False, "stopped": True, "image": None}
        log(f"Item {item}: stopped, arm returned to zero")
        announce("Stopped. I am back at the start.")
        return
    check = after.replace("_after", "_check")
    ok, frac = plate_check.verdict(before, after, check, min_frac=0.003)
    state["last"] = {"item": item, "ok": ok, "frac": round(frac * 100, 1), "image": os.path.basename(check),
                     "err": None if err is None else [round(float(e), 1) for e in err]}
    log(f"Item {item}: {'ON THE PLATE' if ok else 'MISSED'} ({frac:.1%} of plate changed)")
    announce(f"Done. {ITEMS[item].capitalize()} is on the plate." if ok
             else f"I think I missed {ITEMS[item]}. Please check it.")


def run_move(name):
    log(f"Doing the {spoken(name)}")
    rp.replay(*rp.load_dir(os.path.join(MOVES_DIR, name)), DANCE_SPEED, f"move_{name}")
    if rp.STOP.is_set():
        log(f"Move {spoken(name)}: stopped, arm returned to zero")
        announce("Stopped. I am back at the start.")
    else:
        log(f"Move {spoken(name)}: done")


def worker():
    while True:
        with jobs_cv:
            while not jobs:
                state["busy"], state["job"], state["job_kind"] = False, None, None
                jobs_cv.wait()
            action = jobs.popleft()
            state["queue"] = [label(j) for j in jobs]
            state["busy"], state["job"], state["job_kind"] = True, label(action), list(action)
            rp.STOP.clear()
        state["last_action"] = action
        try:
            run_pick(action[1]) if action[0] == "pick" else run_move(action[1])
        except Exception as e:
            log(f"ERROR during {label(action)}: {e}")
            with jobs_cv:
                jobs.clear(); state["queue"] = []
            hint = " The arm's CAN link looks down." if "Network is down" in str(e) else ""
            announce(f"Something went wrong with the arm.{hint} Check the dashboard log.")


threading.Thread(target=worker, daemon=True).start()


# ---------- voice: ElevenLabs ears and mouth ----------

_eleven = None


def eleven():
    global _eleven
    if _eleven is None:
        from elevenlabs.client import ElevenLabs
        _eleven = ElevenLabs(api_key=api_key())
    return _eleven


def tts_b64(text):
    if not text:
        return None
    try:
        mp3 = b"".join(eleven().text_to_speech.convert(
            voice_id=VOICE, text=text, model_id="eleven_flash_v2_5", output_format="mp3_44100_128"))
        return base64.b64encode(mp3).decode()
    except Exception as e:
        log(f"(speech failed: {str(e)[-100:]})")
        return None


# ---------- the agent's brain: Claude with the arm's tools ----------

def system_prompt():
    moves = list_moves()
    if state["busy"]:
        status = f"busy doing {state['job']}" + (f", then {', '.join(state['queue'])}" if state["queue"] else "")
    else:
        status = "idle, ready"
    return (
        "You are the voice of a reBot six-joint robot arm demoing at the Revolute robotics hackathon "
        "at FabLab Kendall in Cambridge. You pick items off the table and drop them on a white plate, "
        "and you can perform recorded moves and dances. Item 1 is a roll of tape, item 2 is a small white "
        "box, item 3 is a snack bar. You learned each pick and each move from one human demonstration "
        "with a leader arm and replay it; a camera checks the plate after a pick.\n"
        f"Moves you know: {', '.join(spoken(m) for m in moves) if moves else 'none yet'}.\n"
        f"Arm status right now: {status}.\n"
        "When someone asks for physical actions, call the tools, in the order asked; several calls queue "
        "up and run one after another. 'Dance' with no specific move means pick one of your moves you like. "
        "'Again' means repeat the last thing you did. Only act on clear requests; if a sentence sounds like "
        "background chatter, just answer briefly. If asked for a move you don't know, say which ones you know. "
        "Your replies are spoken aloud: one or two short, friendly sentences, plain words, no lists, "
        "no markdown, no emoji."
    )


def agent_tools():
    items = sorted(int(k) for k in json.load(open(CHOSEN)))
    tools = [
        {"name": "pick_up_item",
         "description": "Pick one item off the table and drop it on the plate (takes about a minute). "
                        "1 = tape, 2 = small white box, 3 = snack bar.",
         "input_schema": {"type": "object", "properties": {"item": {"type": "integer", "enum": items}},
                          "required": ["item"]}},
        {"name": "stop_arm", "description": "Stop the arm immediately and cancel everything queued.",
         "input_schema": {"type": "object", "properties": {}}},
    ]
    moves = list_moves()       # always offered, so remembered do_move calls stay valid
    name = {"type": "string", "enum": moves} if moves else {"type": "string"}
    tools.append({"name": "do_move",
                  "description": "Perform one recorded move or dance." if moves else "No moves are recorded yet.",
                  "input_schema": {"type": "object", "properties": {"name": name}, "required": ["name"]}})
    return tools


def run_tool(name, args):
    if name == "stop_arm":
        stop_all(); return "Stopped and cleared the queue."
    if name == "pick_up_item":
        action = ("pick", int(args.get("item", 0)))
    elif name == "do_move":
        action = ("move", str(args.get("name", "")))
    else:
        return f"Unknown tool {name}."
    was_busy = state["busy"]
    ok, bad = enqueue([action])
    if bad:
        return f"Could not queue {label(action)}: {bad[0][1]}."
    return f"Queued {label(action)}" + (" (it will start after the current job)." if was_busy else "; starting now.")


history = deque(maxlen=8)   # whole turns (user text, tool calls + results, reply), oldest dropped first
_claude = None
_fast_ids = iter(range(10**9))


def remember(text, calls, reply):
    """Keep a fast-path turn in Claude's memory too, as if it had made the tool calls itself."""
    turn = [{"role": "user", "content": text}]
    if calls:
        ids = [f"toolu_fast{next(_fast_ids)}" for _ in calls]
        turn += [{"role": "assistant", "content": [{"type": "tool_use", "id": i, "name": n, "input": a}
                                                   for i, (n, a, _) in zip(ids, calls)]},
                 {"role": "user", "content": [{"type": "tool_result", "tool_use_id": i, "content": out}
                                              for i, (_, _, out) in zip(ids, calls)]}]
    turn.append({"role": "assistant", "content": reply})
    history.append(turn)


def claude_agent(text):
    """Claude's spoken reply, after it has queued any actions it decided on. None if unavailable."""
    global _claude
    key = env_value("ANTHROPIC_API_KEY")
    if not key:
        return None
    import anthropic
    if _claude is None:
        _claude = anthropic.Anthropic(api_key=key)
    turn = [{"role": "user", "content": text}]
    reply, acted, system = "", [], system_prompt()   # status as it was when you spoke
    try:
        for _ in range(4):
            r = _claude.beta.messages.create(
                model="claude-opus-5-5", max_tokens=1024, system=system,
                messages=[m for t in history for m in t] + turn,
                tools=agent_tools(), output_config={"effort": "low"},
                betas=["server-side-fallback-2026-07-01"], fallbacks="default")
            blocks, results = [], []
            for b in r.content:
                if b.type == "text" and b.text.strip():
                    blocks.append({"type": "text", "text": b.text}); reply = b.text.strip()
                elif b.type == "tool_use":
                    blocks.append({"type": "tool_use", "id": b.id, "name": b.name, "input": b.input})
                    out = run_tool(b.name, b.input or {})
                    acted.append(f"{b.name}({json.dumps(b.input)})")
                    results.append({"type": "tool_result", "tool_use_id": b.id, "content": out})
            turn.append({"role": "assistant", "content": blocks or [{"type": "text", "text": "Okay!"}]})
            if not results:
                break
            turn.append({"role": "user", "content": results})
    except Exception as e:
        log(f"(Claude failed: {str(e)[-120:]})")
        return None
    if acted:
        log("Claude: " + ", ".join(acted))
    reply = reply or "Okay!"
    if turn[-1]["role"] == "user":            # ran out of rounds right after tool results
        turn.append({"role": "assistant", "content": reply})
    history.append(turn)
    return reply


# ---------- fast path: plain commands skip Claude ----------

STOP_RE = re.compile(r"\b(stop|halt|freeze|emergency|abort|cancel)\b")
AGAIN_RE = re.compile(r"\b(again|one more time|repeat|encore|do it again)\b")
PICK_WORDS = re.compile(r"\b(pick|grab|get|fetch|bring|take|move|put|drop)\b")
WAKE_RE = re.compile(r"\b(re-?bot|robot|robo|reebok|reboot|hey bot)\b")
NOISE_RE = re.compile(r"^[\s\W]*((\[[^\]]*\]|\([^)]*\))[\s\W]*)*$")   # "[clicking]", "(silence)", "..."
COMPLEX_RE = re.compile(r"\b(then|and|after|before|twice|times|while|both|all)\b")


def fast_actions(t):
    """The actions a plain command asks for (possibly empty)."""
    acts, moves = [], list_moves()
    for m in moves:
        if re.search(r"\b" + r"[\s_-]*".join(map(re.escape, m.split("_"))) + r"\b", t):
            acts.append(("move", m))
    if not acts and re.search(r"\bdance\b", t) and moves:
        acts.append(("move", random.choice(moves)))
    item = which_item(t)
    if item and (PICK_WORDS.search(t) or len(t.split()) <= 3):
        acts.append(("pick", item))
    if not acts and AGAIN_RE.search(t) and state["last_action"]:
        acts.append(tuple(state["last_action"]))
    return acts


def say_queued(ok, bad, was_busy):
    if bad and not ok:
        return f"Sorry, {bad[0][1]}." + (f" I know {', '.join(map(spoken, list_moves()))}." if list_moves() else "")
    parts = [f"picking up item {a[1]}, {ITEMS[a[1]]}" if a[0] == "pick" else f"doing the {spoken(a[1])}" for a in ok]
    text = ("Okay, after this I'll be " if was_busy else "Okay, ") + " and then ".join(parts) + "."
    return text[0].upper() + text[1:]


def builtin_reply(t):
    moves = list_moves()
    if re.search(r"\b(hi|hello|hey)\b", t):
        return "Hi! I'm the reBot arm. Ask me to pick up item one, two, or three" + (", or to dance." if moves else ".")
    if re.search(r"what can you do|help|how does this work|how do you work", t):
        return ("I pick items off the table and drop them on the plate"
                + (f", and I can do the {', the '.join(map(spoken, moves))}" if moves else "")
                + ". I learned everything from one human demonstration.")
    if re.search(r"\bdance\b", t):
        return "I don't know any dances yet. Record one with teleop dot s h move."
    if re.search(r"which items|what items|what can you pick", t):
        return "Item one is the tape, item two is the small box, and item three is the snack bar."
    if re.search(r"status|are you (ready|busy|ok)", t):
        return f"I'm busy with {state['job']}." if state["busy"] else "I'm ready."
    if re.search(r"\b(thanks|thank you)\b", t):
        return "You're welcome!"
    return "Sorry, I didn't catch that. Say pick up item one, two, or three."


def handle_text(text, hands_free=False):
    """The spoken reply, after queueing whatever was asked for. None = ignore (noise, no wake word)."""
    t = text.lower().strip()
    if not t or NOISE_RE.match(t):
        return None
    if STOP_RE.search(t):                       # always obeyed, wake word or not
        stop_all(); remember(text, [("stop_arm", {}, "Stopped and cleared the queue.")], "Stopping now.")
        return "Stopping now."
    if hands_free and not WAKE_RE.search(t):    # open mic: only act when addressed by name
        return None
    acts = fast_actions(t)
    simple = len(acts) == 1 and len(t.split()) <= 8 and not COMPLEX_RE.search(t)
    if simple:
        return fast_do(text, acts)
    reply = claude_agent(text)
    if reply:
        return reply
    if acts:                                    # no Claude: do what we understood
        return fast_do(text, acts)
    return builtin_reply(t)


def fast_do(text, acts):
    was_busy = state["busy"]
    ok, bad = enqueue(acts)
    reply = say_queued(ok, bad, was_busy)
    calls = [("pick_up_item", {"item": a[1]}, f"Queued {label(a)}.") if a[0] == "pick" else
             ("do_move", {"name": a[1]}, f"Queued {label(a)}.") for a in ok]
    calls += [("pick_up_item", {"item": a[1]}, f"Could not queue: {why}.") if a[0] == "pick" else
              ("do_move", {"name": a[1]}, f"Could not queue: {why}.") for a, why in bad]
    remember(text, calls, reply)
    return reply


# ---------- HTTP ----------

def link_status():
    can = subprocess.run(["ip", "-br", "link", "show", "can0"], capture_output=True, text=True).stdout
    return {
        "follower": "UP" in can.split()[1:2] if can else False,
        "leader": os.access("/dev/ttyUSB0", os.W_OK),
        "camera": cam.get() is not None,
        "voice": bool(env_value("ELEVENLABS_API_KEY")),
        "claude": bool(env_value("ANTHROPIC_API_KEY")),
    }


@app.get("/")
def index():
    return send_file(os.path.join(HERE, "dashboard.html"))


@app.get("/video")
def video():
    def gen():
        while True:
            f = cam.get()
            if f is not None:
                ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
            time.sleep(1 / 15)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")


@app.get("/status")
def status():
    return jsonify(busy=state["busy"], job=state["job"], job_kind=state["job_kind"], queue=state["queue"],
                   last=state["last"], log=list(state["log"]), links=link_status(),
                   items={str(k): v for k, v in ITEMS.items()}, chosen=sorted(json.load(open(CHOSEN))),
                   moves=[{"name": m, "label": spoken(m)} for m in list_moves()],
                   speed=SPEED, audio_id=state["audio_id"], announce=state.get("announce"), phone=phone_info())


@app.get("/announcement")
def announcement():
    return jsonify(audio=state["audio"], text=state.get("announce"), audio_id=state["audio_id"])


@app.post("/pick/<int:item>")
def pick(item):
    ok, bad = enqueue([("pick", item)])
    return jsonify(ok=bool(ok), msg=bad[0][1] if bad else "queued"), (200 if ok else 409)


@app.post("/move/<name>")
def move(name):
    ok, bad = enqueue([("move", name)])
    return jsonify(ok=bool(ok), msg=bad[0][1] if bad else "queued"), (200 if ok else 409)


@app.post("/stop")
def stop():
    stop_all()
    return jsonify(ok=True)


@app.post("/speed")
def speed():
    global SPEED
    SPEED = max(0.25, min(1.0, float(request.json["speed"])))
    log(f"Pick speed set to {SPEED}x")
    return jsonify(speed=SPEED)


def respond(heard, hands_free=False):
    reply = handle_text(heard, hands_free)
    if reply is None:
        return jsonify(heard=heard, reply=None, audio=None, ignored=True)
    log(f'Robot: "{reply}"')
    return jsonify(heard=heard, reply=reply, audio=tts_b64(reply), ignored=False)


@app.post("/chat")
def chat():
    """Typed text in, spoken reply out."""
    text = request.json.get("text", "")
    log(f'You typed: "{text}"')
    return respond(text)


@app.post("/voice")
def voice():
    audio = request.files["audio"]
    hands_free = request.form.get("hands_free") == "1"
    try:
        text = eleven().speech_to_text.convert(
            model_id="scribe_v1", file=(audio.filename or "speech.webm", audio.read(), audio.mimetype),
            language_code="en").text.strip()
    except Exception as e:
        log(f"(listening failed: {str(e)[-100:]})")
        if hands_free:
            return jsonify(heard="", reply=None, audio=None, ignored=True)
        return jsonify(heard="", reply="Sorry, I couldn't hear that.", audio=None, ignored=False)
    if text and not NOISE_RE.match(text.lower()):
        log(f'You said: "{text}"')
    return respond(text, hands_free)


# ---------- phone calls: Cartesia's voice agent calls these as webhook tools (see phone_agent.py) ----------
# Reached through a public tunnel, so every request needs PHONE_TOKEN (Bearer). Callers listed in
# PHONE_ALLOWED (comma-separated, +1...) may act right away; anyone else must say PHONE_PIN first,
# which unlocks that one call. "stop" always works. Replies are plain text for the phone agent.

unlocked_calls, pin_failures = set(), {}
phone_mode = {"on": os.environ.get("PHONE_MODE", "off") == "on"}   # dashboard switch: phone may drive the arm


def phone_info():
    try:
        saved = json.load(open(os.path.join(rp.DATA, "phone.json")))
    except (FileNotFoundError, ValueError):
        saved = {}
    return {"number": saved.get("number"), "pin": env_value("PHONE_PIN"), "on": phone_mode["on"],
            "ready": bool(saved.get("number") and saved.get("public_url")), "url": saved.get("public_url")}
PIN_TRIES = 5


def phone_guard():
    """None if the request may proceed, else a (text, status) response."""
    token = env_value("PHONE_TOKEN")
    if not token or request.headers.get("Authorization", "") != f"Bearer {token}":
        return "unauthorized", 401
    return None


def phone_args():
    j = request.get_json(silent=True) or {}
    return j, str(j.get("call_id") or ""), str(j.get("caller") or "")


def caller_trusted(call_id, caller):
    if not env_value("PHONE_PIN"):   # no PIN configured: every caller may drive (the dashboard switch still applies)
        return True
    allowed = {n.strip() for n in (env_value("PHONE_ALLOWED") or "").split(",") if n.strip()}
    return caller in allowed or call_id in unlocked_calls


def phone_status_text(call_id, caller):
    moves = list_moves()
    arm = (f"Busy doing {state['job']}" + (f"; queued after it: {', '.join(state['queue'])}." if state["queue"] else ".")
           if state["busy"] else "Idle and ready.")
    last = state["last"]
    last_txt = ("" if not last else
                f" Last pick: item {last['item']} " + ("was stopped." if last.get("stopped") else
                                                      ("landed on the plate." if last["ok"] else "missed the plate.")))
    return (f"{arm}{last_txt} Items: 1 tape, 2 small white box, 3 snack bar. "
            f"Moves: {', '.join(map(spoken, moves)) if moves else 'none recorded yet'}. "
            f"Arm connected: {'yes' if link_status()['follower'] else 'NO, the CAN link is down'}. "
            f"Phone control: {'on' if phone_mode['on'] else 'OFF (switched off on the dashboard; only stop works)'}. "
            f"This caller {'may control the arm' if caller_trusted(call_id, caller) else 'must say the PIN first'}.")


@app.post("/phone/status")
def phone_status():
    if (bad := phone_guard()):
        return bad
    _, call_id, caller = phone_args()
    return phone_status_text(call_id, caller)


@app.post("/phone/unlock")
def phone_unlock():
    if (bad := phone_guard()):
        return bad
    j, call_id, caller = phone_args()
    if pin_failures.get(call_id, 0) >= PIN_TRIES:
        return "Locked: too many wrong PINs on this call. Tell the caller to hang up."
    pin = re.sub(r"\D", "", str(j.get("pin", "")))
    if pin and pin == env_value("PHONE_PIN"):
        unlocked_calls.add(call_id)
        log(f"Phone: caller {caller or '?'} unlocked the arm")
        return "PIN correct. This caller may now control the arm for the rest of the call."
    pin_failures[call_id] = pin_failures.get(call_id, 0) + 1
    log(f"Phone: wrong PIN from {caller or '?'} ({pin_failures[call_id]}/{PIN_TRIES})")
    return f"Wrong PIN. {PIN_TRIES - pin_failures[call_id]} tries left."


def phone_enqueue(action, call_id, caller):
    if not phone_mode["on"]:
        return "Phone control is switched off on the robot's dashboard right now. Tell the caller to ask the team to turn it on."
    if not caller_trusted(call_id, caller):
        return "Not allowed yet: ask the caller for the PIN and call unlock_robot first."
    if not link_status()["follower"]:
        return "Can't: the arm is not connected (its CAN link is down). Someone at the robot must run teleop.sh setup."
    was_busy = state["busy"]
    ok, bad = enqueue([action])
    if bad:
        extra = f" Known moves: {', '.join(map(spoken, list_moves())) or 'none'}." if action[0] == "move" else ""
        return f"Could not do that: {bad[0][1]}.{extra}"
    log(f"Phone ({caller or '?'}): {label(action)}")
    return (f"Queued {label(action)}; it starts after the current job." if was_busy
            else f"Started {label(action)}. " + ("A pick takes about a minute." if action[0] == "pick" else ""))


@app.post("/phone/pick")
def phone_pick():
    if (bad := phone_guard()):
        return bad
    j, call_id, caller = phone_args()
    try:
        item = int(j.get("item"))
    except (TypeError, ValueError):
        return "Which item? 1 tape, 2 small box, 3 snack bar."
    return phone_enqueue(("pick", item), call_id, caller)


@app.post("/phone/move")
def phone_move():
    if (bad := phone_guard()):
        return bad
    j, call_id, caller = phone_args()
    name = re.sub(r"[^a-z0-9_]", "", str(j.get("name", "")).lower().replace(" ", "_").replace("-", "_"))
    moves = list_moves()
    if name in ("dance", "any", "") and moves:
        name = random.choice(moves)
    return phone_enqueue(("move", name), call_id, caller)


@app.post("/phone/mode")     # from the dashboard page (local only, not via the agent)
def phone_mode_set():
    if request.headers.get("Authorization"):   # tunnel requests carry the token; this switch is local-only
        return "forbidden", 403
    phone_mode["on"] = bool((request.get_json(silent=True) or {}).get("on"))
    log(f"Phone control switched {'ON' if phone_mode['on'] else 'OFF'}")
    return jsonify(phone_info())


@app.post("/phone/stop")
def phone_stop():
    if (bad := phone_guard()):
        return bad
    _, _, caller = phone_args()
    stop_all()
    log(f"Phone ({caller or '?'}): STOP")
    return "Stopped. The arm is returning to its start position and the queue is cleared."


@app.get("/image/<path:name>")
def image(name):
    return send_file(os.path.join(rp.OUT, os.path.basename(name)))


if __name__ == "__main__":
    log(f"Dashboard on http://localhost:{PORT}")
    app.run(host="127.0.0.1", port=PORT, threaded=True)

"""Call the robot on the phone: a Cartesia voice agent whose tools drive the arm.

  ./phone.sh                           tunnel with cloudflared, then point the agent at it
  PUBLIC_URL=https://... ./phone.sh    use a public URL you already have (ngrok, etc.)

Needs the dashboard running (./dashboard.sh) and, in .env:
  CARTESIA_API_KEY    Cartesia key
  PHONE_TOKEN         shared secret the agent's tool calls must carry (generated)
  PHONE_PIN           4 digits a caller says to unlock the arm for that call (generated)
  PHONE_ALLOWED       optional: +1... numbers (comma-separated) that skip the PIN
  PHONE_NUMBER_ID     optional: which Cartesia number to use (ap_...); else the saved/first free one

Cartesia runs the call (speech in/out + LLM) on its cloud. When the caller asks for something,
the LLM calls a webhook tool -> the tunnel -> the dashboard's /phone/* endpoints -> the arm's queue.
Agent, tool and number ids are saved in data/phone.json so reruns update rather than duplicate.
A quick tunnel's URL changes every run; this script re-points the tools each time.
"""
import json, os, re, shutil, signal, subprocess, sys, threading, time
import socket
import requests
import urllib3.util.connection

# This network's IPv6 is broken: requests would wait out an IPv6 timeout before every call. Use IPv4.
urllib3.util.connection.allowed_gai_family = lambda: socket.AF_INET
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
SAVED = os.path.join(HERE, "data", "phone.json")
API = "https://api.cartesia.ai"
VERSION = "2026-08-14"
LOCAL = f"http://127.0.0.1:{os.environ.get('PORT', '8765')}"
MODEL = os.environ.get("PHONE_MODEL", "claude-sonnet-5")
VOICE_ID = os.environ.get("PHONE_VOICE", "47c38ca4-5f35-497b-b1a3-415245fb35e1")   # "Daniel"
AGENT_NAME = "reBot Arm (phone)"


def env(name, default=None):
    if os.environ.get(name):
        return os.environ[name]
    for line in open(os.path.join(HERE, ".env")):
        if line.startswith(name + "="):
            return line.split("=", 1)[1].strip().strip('"')
    return default


KEY, TOKEN, PIN = env("CARTESIA_API_KEY"), env("PHONE_TOKEN"), env("PHONE_PIN")
if not (KEY and TOKEN):
    sys.exit("Need CARTESIA_API_KEY and PHONE_TOKEN in .env")
H = {"X-API-Key": KEY, "Cartesia-Version": VERSION, "Content-Type": "application/json"}


def api(method, path, body=None):
    r = requests.request(method, API + path, headers=H, json=body, timeout=900)   # creating things can take minutes
    if r.status_code >= 300:
        sys.exit(f"Cartesia {method} {path} failed: HTTP {r.status_code}\n{r.text[:800]}")
    return r.json() if r.text else {}


def save(saved):
    json.dump(saved, open(SAVED, "w"), indent=2)


# ---------- public URL ----------

def start_tunnel():
    exe = shutil.which("cloudflared") or os.path.join(HERE, "bin", "cloudflared")
    if not os.access(exe, os.X_OK):
        sys.exit("No public URL. Install cloudflared (see README in phone.sh) or set PUBLIC_URL=https://...")
    # http2 (TCP 443): venue Wi-Fi often blocks cloudflared's default QUIC on UDP 7844
    p = subprocess.Popen([exe, "tunnel", "--no-autoupdate", "--protocol", "http2", "--url", LOCAL],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    t0, url = time.time(), None
    for line in p.stdout:
        m = re.search(r"https://[a-z0-9-]+\.trycloudflare\.com", line)
        if m and not url:
            url = m.group(0)
            print(f"Tunnel URL {url}, waiting for it to connect...", flush=True)
        if url and "Registered tunnel connection" in line:
            threading.Thread(target=drain, args=(p,), daemon=True).start()
            return p, url
        if time.time() - t0 > 120:
            break
    p.kill()
    sys.exit("cloudflared could not connect a tunnel (network blocking it?). Try another network or PUBLIC_URL=...")


def drain(p):
    """Keep reading cloudflared's log so its pipe never fills (which would stall the tunnel)."""
    for line in p.stdout:
        if re.search(r"\bERR\b", line) and "quic" not in line.lower():
            print("cloudflared:", line.strip(), flush=True)


def wait_reachable(url):
    for _ in range(60):          # a fresh quick-tunnel hostname can take a minute to resolve
        try:
            r = requests.post(url + "/phone/status", json={}, timeout=10,
                              headers={"Authorization": f"Bearer {TOKEN}"})
            if r.status_code == 200:
                return
        except requests.RequestException:
            pass
        time.sleep(3)
    sys.exit(f"{url}/phone/status is not reachable. Is the dashboard running (./dashboard.sh)?")


# ---------- tools ----------

CALL = {   # filled by Cartesia from the live call, not by the model
    "call_id": {"type": "string", "dynamic_variable": "system__conversation_id"},
    "caller": {"type": "string", "dynamic_variable": "system__caller_id"},
}


def tool_defs(url):
    def tool(name, path, description, props=None, required=(), speech="off", timeout=15):
        return {
            "type": "webhook", "name": name, "description": description,
            "pre_tool_speech": speech, "execution_mode": "immediate", "response_timeout_secs": timeout,
            "api_schema": {
                "url": url + path, "method": "POST",
                "request_body_schema": {"type": "object", "properties": {**(props or {}), **CALL},
                                        "required": list(required)},
                "authentication": {"mode": "bearer", "token": {"type": "secret", "secret_value": TOKEN}},
            },
        }
    return [
        tool("robot_status", "/phone/status",
             "What the arm is doing now, what it can pick, which moves/dances it knows, whether it is "
             "connected, and whether this caller may control it. Call at the start of the call and "
             "whenever you need fresh information."),
        tool("unlock_robot", "/phone/unlock",
             "Check the PIN the caller says. Needed before picks or moves unless robot_status says "
             "the caller may already control the arm.",
             {"pin": {"type": "string", "description": "The digits the caller said, e.g. '4821'."}}, ["pin"]),
        tool("pick_up_item", "/phone/pick",
             "Pick one item off the table and drop it on the plate. 1 = tape, 2 = small white box, "
             "3 = snack bar. Several calls queue up in order.",
             {"item": {"type": "integer", "enum": [1, 2, 3], "description": "Item number."}}, ["item"]),
        tool("do_move", "/phone/move",
             "Perform a recorded move or dance by name (from robot_status). Use 'dance' for a random one.",
             {"name": {"type": "string", "description": "Move name, e.g. 'wave' or 'robot dance'."}}, ["name"]),
        tool("stop_arm", "/phone/stop",
             "Stop the arm immediately and cancel everything queued. Anyone may use this; use it the "
             "moment the caller says stop, wait, or sounds alarmed.", speech="off"),
    ]


def sync_tools(saved, url):
    ids = saved.setdefault("tools", {})
    defs = tool_defs(url)
    if len(ids) < len(defs):   # adopt same-named webhook tools left by an earlier, interrupted run
        for t in api("GET", "/v1/agents/tools").get("data", []):
            if t.get("type") == "webhook" and t["name"] in {d["name"] for d in defs}:
                ids.setdefault(t["name"], t["id"])
    def one(t):   # the API is slow (~1 min per call), so all tools go in parallel
        if t["name"] in ids:
            api("PATCH", f"/v1/agents/tools/{ids[t['name']]}", {k: v for k, v in t.items() if k != "type"})
        else:
            ids[t["name"]] = api("POST", "/v1/agents/tools", t)["id"]
    with ThreadPoolExecutor(len(defs)) as pool:
        list(pool.map(one, defs))
    save(saved)
    return [{"id": i} for i in ids.values()]


# ---------- agent ----------

INSTRUCTIONS = """You are the phone voice of reBot, a six-joint robot arm demoing at the Revolute robotics \
hackathon at FabLab Kendall in Cambridge. People call you to make the arm do things. You can pick items off \
the table and drop them on a white plate (item 1 roll of tape, item 2 small white box, item 3 snack bar) and \
perform recorded moves and dances. You learned each one from a single human demonstration.

How to run the call:
- Call robot_status first thing, quietly, before you greet them in detail.
- Only if robot_status says the caller must say the PIN: ask for it and pass the digits to unlock_robot; never \
say the PIN yourself. Otherwise never mention a PIN. If robot_status says phone control is off, explain that the \
team has switched it off on the dashboard (stop still works).
- When asked for actions, call pick_up_item / do_move in the order asked; they queue and run one by one. \
"Dance" without a name means do_move with name "dance". Tell the caller what started and roughly how long \
(a pick is about a minute). You can check progress with robot_status.
- If the caller says stop, wait, no, or sounds worried, call stop_arm immediately, before anything else.
- If the arm is not connected, say someone at the robot needs to reconnect it.
- This is a phone call: one or two short, friendly sentences at a time, plain words, no lists, no markdown.
- When the caller is done or says goodbye, say bye and end the call."""


def sync_agent(saved, tools):
    config = {
        "instructions": INSTRUCTIONS,
        "initial_message": "Hi, this is reBot, the robot arm at FabLab Kendall. What should I do?",
        "model": {"id": MODEL},
        "audio": {"output": {"voice_id": VOICE_ID}, "input": {"noise_suppression": "auto",
                                                              "keyterms": ["reBot", "PIN", "tape", "snack bar"]}},
        "language": {"primary": "en"},
        "tools": tools,
        "system_tools": {"end_call": {}},
        "turn": {"inactivity_check_in_secs": 20, "inactivity_end_call_secs": 90},
    }
    if saved.get("agent_id"):
        api("PATCH", f"/v1/agents/{saved['agent_id']}", {"config": config})
    else:
        saved["agent_id"] = api("POST", "/v1/agents", {
            "name": AGENT_NAME, "description": "Phone control for the reBot arm (webhook tools via tunnel)",
            "config": config})["id"]
        save(saved)


def sync_number(saved):
    numbers = api("GET", "/agents/phone-numbers")["data"]
    want = env("PHONE_NUMBER_ID") or saved.get("number_id")
    if not want:   # first free number on the account; never steal one another agent uses
        free = [n for n in numbers if not n.get("agent")]
        if not free:
            sys.exit("No free Cartesia number. Set PHONE_NUMBER_ID, or provision one: "
                     "cartesia phone-numbers provision reBot")
        want = free[0]["id"]
    num = next((n for n in numbers if n["id"] == want), None)
    if not num:
        sys.exit(f"Phone number {want} not found on this Cartesia account")
    owner = (num.get("agent") or {}).get("id")
    if owner and owner != saved["agent_id"]:
        sys.exit(f"{num['number']} is assigned to another agent ({num['agent'].get('name')}); not taking it")
    if owner != saved["agent_id"]:
        api("PATCH", f"/agents/phone-numbers/{want}", {"agent_id": saved["agent_id"]})
    saved["number_id"], saved["number"] = want, num["number"]


def main():
    saved = json.load(open(SAVED)) if os.path.exists(SAVED) else {}
    tunnel = None
    url = os.environ.get("PUBLIC_URL", "").rstrip("/")
    if not url:
        tunnel, url = start_tunnel()
    print(f"Public URL: {url}")
    if tunnel:   # never leave a tunnel open if setup fails or we're killed
        import atexit; atexit.register(tunnel.terminate)
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    wait_reachable(url)
    saved["public_url"] = url
    print("Setting up the Cartesia agent (the API can be slow, up to a few minutes)...", flush=True)
    tools = sync_tools(saved, url)
    sync_agent(saved, tools)
    sync_number(saved)
    save(saved)
    allowed = env("PHONE_ALLOWED", "")
    print(f"\n  Call {saved['number']} to talk to the robot.")
    print(f"  PIN: {PIN}" + (f"   (no PIN needed from {allowed})" if allowed else "") if PIN else
          "  No PIN: anyone who calls can drive the arm while the dashboard's Phone switch is on.")
    print(f"  Agent: https://play.cartesia.ai/agents/{saved['agent_id']}\n")
    if not tunnel:
        return
    print("Keep this running during calls. Ctrl+C closes the tunnel (calls then can't reach the arm).")
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        tunnel.wait()
    except KeyboardInterrupt:
        pass
    finally:
        tunnel.terminate()
        saved["public_url"] = None; save(saved)   # dashboard shows the phone line as offline


if __name__ == "__main__":
    main()

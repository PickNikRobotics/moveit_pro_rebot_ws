"""Voice control: say "pick up item 2" (or "the tape") and the arm does it.

  python voice_pickup.py              press ENTER, speak for 4 s, the robot answers
  python voice_pickup.py --type       no microphone: type the command instead
  python voice_pickup.py --speed 0.75

Hears with ElevenLabs speech-to-text, answers with ElevenLabs text-to-speech.
Needs ELEVENLABS_API_KEY in the environment or in ~/rebot_lerobot/.env.
"""
import argparse, os, re, subprocess, tempfile, wave
from elevenlabs.client import ElevenLabs
from pickup import pickup

MIC = "plughw:1,6"          # laptop's built-in microphone
SPEAKER = "plughw:1,0"      # laptop speakers
LISTEN_S = 4
VOICE = "JBFqnCBsd6RMkjVDRZzb"
ITEMS = {1: "the tape", 2: "the small box", 3: "the snack bar"}
# Words that name an item, besides its number.
NAMES = {1: ("tape", "roll"), 2: ("box", "cube", "white"), 3: ("snack", "bar", "packet", "granola")}
NUMBERS = {"one": 1, "won": 1, "first": 1, "two": 2, "to": 2, "too": 2, "second": 2,
           "three": 3, "tree": 3, "third": 3}

def api_key():
    if os.environ.get("ELEVENLABS_API_KEY"):
        return os.environ["ELEVENLABS_API_KEY"]
    env = os.path.expanduser("~/rebot_lerobot/.env")
    if os.path.exists(env):
        for line in open(env):
            if line.startswith("ELEVENLABS_API_KEY="):
                return line.split("=", 1)[1].strip().strip('"')
    raise SystemExit("Set ELEVENLABS_API_KEY (or put it in ~/rebot_lerobot/.env)")

def which_item(text):
    """Item number named in a sentence, or None. "stop"/"quit" returns 0."""
    t = text.lower()
    if re.search(r"\b(quit|exit|goodbye|stop listening)\b", t):
        return 0
    m = re.search(r"\bitem\s*(?:number\s*)?(\w+)", t)
    if m:
        w = m.group(1)
        if w.isdigit() and int(w) in ITEMS: return int(w)
        if w in NUMBERS: return NUMBERS[w]
    for n, words in NAMES.items():
        if any(re.search(rf"\b{w}\b", t) for w in words): return n
    for w in re.findall(r"\w+", t):
        if w.isdigit() and int(w) in ITEMS: return int(w)
        if w in NUMBERS and w not in ("to", "too"): return NUMBERS[w]
    return None

class Voice:
    def __init__(self):
        self.client = ElevenLabs(api_key=api_key())

    def listen(self):
        with tempfile.NamedTemporaryFile(suffix=".wav") as f:
            subprocess.run(["arecord", "-q", "-D", MIC, "-f", "S16_LE", "-r", "16000", "-c", "1",
                            "-d", str(LISTEN_S), f.name], check=True)
            with open(f.name, "rb") as audio:
                return self.client.speech_to_text.convert(model_id="scribe_v1", file=audio,
                                                          language_code="en").text

    def say(self, text):
        print(f"ROBOT: {text}")
        try:
            pcm = b"".join(self.client.text_to_speech.convert(
                voice_id=VOICE, text=text, model_id="eleven_flash_v2_5", output_format="pcm_16000"))
            with tempfile.NamedTemporaryFile(suffix=".wav") as f:
                with wave.open(f.name, "wb") as w:
                    w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(pcm)
                subprocess.run(["aplay", "-q", "-D", SPEAKER, f.name], timeout=30)
        except Exception as e:          # a voice hiccup must not stop the robot
            print(f"(ElevenLabs speech failed, using the laptop voice: {str(e)[-120:]})")
            subprocess.run(["spd-say", "--wait", text], timeout=30)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--type", action="store_true", help="type commands instead of speaking")
    a = ap.parse_args()
    v = Voice()
    v.say("Ready. Which item should I pick up?")
    while True:
        if a.type:
            heard = input("\nYou: ")
        else:
            if input(f"\nPress ENTER and speak ({LISTEN_S} s), or q + ENTER to quit: ").strip() == "q":
                break
            print("Listening...")
            try:
                heard = v.listen()
            except Exception as e:
                print(f"(listening failed: {str(e)[-120:]})")
                heard = input("Type the command instead: ")
            print(f"You said: {heard!r}")
        item = which_item(heard)
        if item == 0:
            v.say("Goodbye."); break
        if item is None:
            v.say("Sorry, which item? Say item one, two, or three."); continue
        v.say(f"Picking up item {item}, {ITEMS[item]}.")
        ok = pickup(item, a.speed)
        v.say(f"Done. {ITEMS[item].capitalize()} is on the plate." if ok
              else f"I think I missed {ITEMS[item]}. Please check it.")

if __name__ == "__main__":
    main()

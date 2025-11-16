import sounddevice as sd
import queue
import vosk
import json
import re
import difflib

# -------------------------
# 1. 配置参数
# -------------------------
model_path = r"vosk-model-small-en-us-0.15"
samplerate = 16000
device = None  # 默认麦克风

# -------------------------
# 2. 创建音频流队列
# -------------------------
q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print(status, flush=True)
    q.put(bytes(indata))

# -------------------------
# 3. 加载模型
# -------------------------
print("Loading model (may take 30s)...")
model = vosk.Model(model_path)
recognizer = vosk.KaldiRecognizer(model, samplerate)

# -------------------------
# 4. 坐标映射表
# -------------------------
mapping = {
    "one one": "A1",
    "one two": "A2",
    "one three": "A3",

    "two one": "B1",
    "two two": "B2",
    "two three": "B3",

    "three one": "C1",
    "three two": "C2",
    "three three": "C3",
}

# -------------------------
# 5. 文本清理函数
# -------------------------
def clean_text(text: str) -> str:
    text = text.lower().strip()

    # 去掉单词结尾重复 (twowo → two)
    text = re.sub(r"(\b\w+)\1\b", r"\1", text)

    # 常见错音替换
    text = text.replace("choo", "two")  # "choo choo" → "two two"

    # 多余空格清理
    text = re.sub(r"\s+", " ", text)
    return text.strip()

# -------------------------
# 6. 模糊匹配函数
# -------------------------
def fuzzy_match(text, mapping, cutoff=0.75):
    cleaned = clean_text(text)
    best = difflib.get_close_matches(cleaned, mapping.keys(), n=1, cutoff=cutoff)
    if best:
        return best[0], mapping[best[0]]
    return None, None

# -------------------------
# 7. 实时识别与匹配
# -------------------------
# print("🎤 Speak English now (Ctrl+C to stop)")

def listen_for_move():
    print("🎤 Please speak your move (e.g., 'one two')")

    with sd.RawInputStream(samplerate=samplerate, blocksize=8000, device=device,
                           dtype='int16', channels=1, callback=callback):
        while True:
            data = q.get()

            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "").strip().lower()

                if text:
                    print("🗣️ Heard:", text)

                    # 尝试模糊匹配
                    match_text, coord = fuzzy_match(text, mapping)

                    if coord:
                        print(f"✅ Matched '{match_text}' → coordinate {coord}")
                        return coord  # ⭐ 直接返回坐标 (row, col)
                    else:
                        print("❌ No valid match, please say again…")
            else:
                # partial result 不需要用
                pass
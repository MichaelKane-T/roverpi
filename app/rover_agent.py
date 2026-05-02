"""
rover_agent.py
RoverPi RL Agent — Pi Zero 2W edition

Policy network:  tiny MLP (13 → 32 → 16 → 5)
Training:        on-device, background thread, small batches
Inference:       TFLite (quantised INT8) for speed
Experience:      ring buffer, 5000 transitions max
Exploration:     epsilon-greedy, decays over time
Persistence:     model + buffer checkpointed to disk
"""

import os
import time
import threading
import collections
import random
import numpy as np
import requests

# TFLite is part of tflite-runtime on Pi — much lighter than full TF
try:
    from tflite_runtime.interpreter import Interpreter
    TFLITE_AVAILABLE = True
except ImportError:
    TFLITE_AVAILABLE = False
    print("[Agent] tflite_runtime not found — inference disabled, training only")

# Keras / TF for training (installed separately, runs in background)
try:
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
    import tensorflow as tf
    from tensorflow import keras
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False
    print("[Agent] TensorFlow not found — using random policy")

# ── constants ────────────────────────────────────────────────────────────────
OBS_DIM          = 13
N_ACTIONS        = 5
BUFFER_SIZE      = 5_000
BATCH_SIZE       = 32
TRAIN_EVERY      = 50        # train after every N steps collected
GAMMA            = 0.95      # discount factor
LR               = 1e-3
EPSILON_START    = 0.15
EPSILON_END      = 0.02
EPSILON_DECAY    = 0.995     # multiplied each episode
MODEL_PATH       = "/home/pi/roverpi/model.keras"
TFLITE_PATH      = "/home/pi/roverpi/model.tflite"
BUFFER_PATH      = "/home/pi/roverpi/replay_buffer.npy"
CHECKPOINT_EVERY = 200       # steps between saves


# ── replay buffer ────────────────────────────────────────────────────────────

class ReplayBuffer:
    def __init__(self, maxlen=BUFFER_SIZE):
        self._buf = collections.deque(maxlen=maxlen)

    def push(self, obs, action, reward, next_obs):
        self._buf.append((obs, action, reward, next_obs))

    def sample(self, n):
        batch = random.sample(self._buf, min(n, len(self._buf)))
        obs, acts, rews, nobs = zip(*batch)
        return (
            np.array(obs,  dtype=np.float32),
            np.array(acts, dtype=np.int32),
            np.array(rews, dtype=np.float32),
            np.array(nobs, dtype=np.float32),
        )

    def __len__(self):
        return len(self._buf)

    def save(self, path):
        arr = np.array(list(self._buf), dtype=object)
        np.save(path, arr, allow_pickle=True)

    def load(self, path):
        if not os.path.exists(path):
            return
        arr = np.load(path, allow_pickle=True)
        for item in arr:
            self._buf.append(tuple(item))
        print(f"[Buffer] Loaded {len(self._buf)} transitions from {path}")


# ── policy network ───────────────────────────────────────────────────────────

def build_model():
    """Tiny MLP — fast enough to train on Zero 2W in background."""
    if not TF_AVAILABLE:
        return None
    inp = keras.Input(shape=(OBS_DIM,))
    x   = keras.layers.Dense(32, activation="relu")(inp)
    x   = keras.layers.Dense(16, activation="relu")(x)
    out = keras.layers.Dense(N_ACTIONS, activation="linear")(x)
    model = keras.Model(inp, out)
    model.compile(optimizer=keras.optimizers.Adam(LR), loss="mse")
    return model


def export_tflite(keras_model, path=TFLITE_PATH):
    """Convert trained Keras model → quantised TFLite for fast inference."""
    if not TF_AVAILABLE:
        return
    converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]   # INT8 quantisation
    tflite_model = converter.convert()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(tflite_model)
    print(f"[Agent] TFLite model exported to {path}")


# ── main agent class ─────────────────────────────────────────────────────────

class RoverAgent:
    CLOUD_URL = "http://192.168.1.185:7000/predict"
    USE_CLOUD = True

    def __init__(self):
        self.buffer   = ReplayBuffer()
        self.epsilon  = EPSILON_START
        self.steps    = 0
        self._lock    = threading.Lock()
        self._training = False

        # Keras model (used for training in background thread)
        self.model = build_model()
        if self.model and os.path.exists(MODEL_PATH):
            self.model.load_weights(MODEL_PATH)
            print(f"[Agent] Loaded weights from {MODEL_PATH}")

        # TFLite interpreter (used for fast inference in main thread)
        self.interpreter = None
        self._load_tflite()

        # Load replay buffer from disk if available
        self.buffer.load(BUFFER_PATH)

        # Background training thread
        self._train_thread = threading.Thread(
            target=self._train_loop, daemon=True
        )
        self._train_thread.start()

    # ── inference ────────────────────────────────────────────────────────────

    def select_action(self, obs):
        """
        Choose action using cloud model first.
        Falls back to random/local policy if cloud fails.
        """

        if self.USE_CLOUD:
            try:
                # Build same 21-feature shape used during training.
                features = [float(x) for x in obs] + [
                    200.0,  # dist_cm placeholder for now
                    1,      # path_clear placeholder
                    0,      # direction placeholder
                    0,      # fault placeholder
                    0.0,    # yaw_sin placeholder
                    1.0,    # yaw_cos placeholder
                    0.0,    # gz_dps placeholder
                    float(getattr(self, "last_action", 4)),
                ]

                r = requests.post(
                    self.CLOUD_URL,
                    json={"features": features},
                    timeout=0.25,
                )

                if r.ok:
                    action = int(r.json()["action"])
                    self.last_action = action
                    return action

            except Exception as e:
                print(f"[Agent] Cloud inference failed: {e}")

        # fallback
        action = self._random_action()
        self.last_action = action
        return action

    def _tflite_predict(self, obs: np.ndarray) -> int:
        inp_detail = self.interpreter.get_input_details()[0]
        out_detail = self.interpreter.get_output_details()[0]
        self.interpreter.set_tensor(inp_detail["index"], obs[None].astype(np.float32))
        self.interpreter.invoke()
        q = self.interpreter.get_tensor(out_detail["index"])[0]
        return int(np.argmax(q))
    
    def _random_action(self) -> int:
        # Bias toward forward while still allowing exploration
        return random.choices(
            population=[0, 1, 2, 3, 4],
            weights=[0.65, 0.12, 0.12, 0.03, 0.08],
            k=1,
        )[0]

    # ── experience storage ────────────────────────────────────────────────────

    def store(self, obs, action, reward, next_obs):
        self.buffer.push(obs, action, reward, next_obs)
        self.steps += 1

    # ── epsilon decay ─────────────────────────────────────────────────────────

    def decay_epsilon(self):
        self.epsilon = max(EPSILON_END, self.epsilon * EPSILON_DECAY)

    # ── background training loop ──────────────────────────────────────────────

    def _train_loop(self):
        """
        Runs in a daemon thread. Trains whenever there's enough data.
        Sleeps between batches to avoid choking the Pi Zero 2W CPU.
        """
        while True:
            time.sleep(2.0)   # yield to inference / Flask

            if not TF_AVAILABLE or self.model is None:
                continue
            if len(self.buffer) < BATCH_SIZE:
                continue
            if self.steps % TRAIN_EVERY != 0:
                continue

            self._training = True
            try:
                self._train_step()
            except Exception as e:
                print(f"[Agent] Train error: {e}")
            finally:
                self._training = False

            # checkpoint periodically
            if self.steps % CHECKPOINT_EVERY == 0:
                self._save()

    def _train_step(self):
        obs, acts, rews, nobs = self.buffer.sample(BATCH_SIZE)

        with self._lock:
            # Q(s, a) targets using one-step Bellman
            q_next   = self.model.predict(nobs, verbose=0)
            q_target = self.model.predict(obs,  verbose=0)

        for i in range(len(acts)):
            q_target[i, acts[i]] = rews[i] + GAMMA * np.max(q_next[i])

        with self._lock:
            self.model.fit(obs, q_target, epochs=1, verbose=0, batch_size=BATCH_SIZE)

        # re-export TFLite so inference picks up latest weights
        export_tflite(self.model, TFLITE_PATH)
        self._load_tflite()

    # ── persistence ───────────────────────────────────────────────────────────

    def _save(self):
        os.makedirs(os.path.dirname(MODEL_PATH), exist_ok=True)
        if self.model:
            self.model.save_weights(MODEL_PATH)
        self.buffer.save(BUFFER_PATH)
        print(f"[Agent] Checkpoint saved at step {self.steps}")

    def _load_tflite(self):
        if not TFLITE_AVAILABLE or not os.path.exists(TFLITE_PATH):
            return
        try:
            self.interpreter = Interpreter(model_path=TFLITE_PATH)
            self.interpreter.allocate_tensors()
            print(f"[Agent] TFLite interpreter loaded from {TFLITE_PATH}")
        except Exception as e:
            print(f"[Agent] TFLite load error: {e}")
            self.interpreter = None

    @property
    def is_training(self):
        return self._training
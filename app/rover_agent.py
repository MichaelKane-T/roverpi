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
EPSILON_START    = 1.0
EPSILON_END      = 0.10
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

    def select_action(self, obs: np.ndarray) -> int:
        """Epsilon-greedy action selection using TFLite interpreter."""
        if random.random() < self.epsilon:
            return random.randint(0, N_ACTIONS - 1)

        if self.interpreter is not None:
            return self._tflite_predict(obs)

        if self.model is not None:
            with self._lock:
                q = self.model.predict(obs[None], verbose=0)[0]
            return int(np.argmax(q))

        return random.randint(0, N_ACTIONS - 1)

    def _tflite_predict(self, obs: np.ndarray) -> int:
        inp_detail = self.interpreter.get_input_details()[0]
        out_detail = self.interpreter.get_output_details()[0]
        self.interpreter.set_tensor(inp_detail["index"], obs[None].astype(np.float32))
        self.interpreter.invoke()
        q = self.interpreter.get_tensor(out_detail["index"])[0]
        return int(np.argmax(q))

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
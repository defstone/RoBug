# Edge Impulse Linux Python Runtime — Developer Handbook  
*A practical guide to running `.eim` models on embedded Linux devices - compiled by Copilot*

---

## Table of Contents
1. [Introduction](#introduction)  
2. [Architecture Overview](#architecture-overview)  
   - 2.1 [Deployment Bundle Contents](#deployment-bundle-contents)  
   - 2.2 [Runtime Components](#runtime-components)  
3. [Python API Reference](#python-api-reference)  
   - 3.1 [ImpulseRunner](#impulserunner)  
   - 3.2 [ImageImpulseRunner](#imageimpulserunner)  
   - 3.3 [AudioImpulseRunner](#audioimpulserunner)  
4. [Inference Workflows](#inference-workflows)  
   - 4.1 [Image Classification](#image-classification)  
   - 4.2 [Object Detection](#object-detection)  
   - 4.3 [Audio Classification](#audio-classification)  
   - 4.4 [Continuous Inference Loop](#continuous-inference-loop)  
5. [Best Practices](#best-practices)  
6. [Troubleshooting](#troubleshooting)  
7. [Appendix](#appendix)  

---

# Introduction

Edge Impulse’s Linux deployment format (`.eim`) bundles a trained model, DSP pipeline, metadata, and a C++ inference engine. The Python runtime is a thin wrapper around this engine, enabling fast inference on embedded Linux devices such as Raspberry Pi, Jetson Nano, and ARM SBCs.

This handbook provides a complete, developer‑friendly explanation of how the runtime works and how to use it effectively.

---

# Architecture Overview

## Deployment Bundle Contents

A typical Linux deployment export contains:

```
model.eim
libedgeimpulse.so
edge_impulse_linux.py
```

### `model.eim`
A binary container holding:
- DSP configuration  
- Neural network weights  
- Labels  
- Model metadata  
- Preprocessing parameters  

### `libedgeimpulse.so`
A compiled C++ inference engine that:
- Loads the `.eim` file  
- Allocates input/output buffers  
- Runs DSP blocks  
- Executes the neural network  
- Measures timing  

### `edge_impulse_linux.py`
A thin Python wrapper exposing:
- `ImpulseRunner`  
- `ImageImpulseRunner`  
- `AudioImpulseRunner`  

---

# Runtime Components

The Python layer does not perform inference itself. It forwards calls to the C++ engine, which handles:

- Memory allocation  
- DSP preprocessing  
- Neural network execution  
- Timing measurement  
- Result formatting  

This architecture ensures high performance even on low‑power ARM boards.

---

# Python API Reference

## ImpulseRunner

Used for raw‑feature models (accelerometer, tabular data, custom DSP).

### Constructor
```python
runner = ImpulseRunner(path_to_eim)
```

### Methods

| Method | Description |
|--------|-------------|
| `init()` | Loads the model and returns metadata. |
| `get_model_parameters()` | Returns DSP + NN parameters. |
| `get_input_frame_size()` | Number of raw features expected. |
| `classify(features)` | Runs inference on a 1D feature vector. |
| `stop()` | Frees C++ buffers. |

### Example
```python
from edge_impulse_linux.runner import ImpulseRunner

runner = ImpulseRunner("model.eim")
info = runner.init()

frame_size = info['model_parameters']['input_features_count']
features = [0.1] * frame_size

result = runner.classify(features)
print(result)

runner.stop()
```

---

## ImageImpulseRunner

Used for image classification and object detection.

### Constructor
```python
runner = ImageImpulseRunner(path_to_eim)
```

### Methods

| Method | Description |
|--------|-------------|
| `init()` | Loads model and returns metadata. |
| `get_input_width()` | Required image width. |
| `get_input_height()` | Required image height. |
| `get_input_channels()` | 1 or 3. |
| `get_features_from_image(img)` | Manual preprocessing. |
| `get_features_from_image_auto_studio_settings(img)` | Studio‑identical preprocessing. |
| `classify(features)` | Runs inference. |
| `stop()` | Frees resources. |

### Example
```python
from edge_impulse_linux.image import ImageImpulseRunner
import cv2

runner = ImageImpulseRunner("model.eim")
info = runner.init()

w = info['model_parameters']['image_input_width']
h = info['model_parameters']['image_input_height']

img = cv2.imread("image.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

features, cropped = runner.get_features_from_image_auto_studio_settings(img)
result = runner.classify(features)

print(result)
runner.stop()
```

---

## AudioImpulseRunner

Used for audio classification and anomaly detection.

### Constructor
```python
runner = AudioImpulseRunner(path_to_eim)
```

### Methods

| Method | Description |
|--------|-------------|
| `init()` | Loads model and returns audio parameters. |
| `get_sample_length_ms()` | Window size in ms. |
| `get_sample_frequency()` | Sample rate. |
| `get_input_frame_size()` | Number of samples per frame. |
| `classify(audio_frame)` | Runs inference on PCM samples. |
| `stop()` | Frees resources. |

### Example
```python
from edge_impulse_linux.audio import AudioImpulseRunner
import numpy as np

runner = AudioImpulseRunner("model.eim")
info = runner.init()

frame_size = info['model_parameters']['input_features_count']
audio = np.zeros(frame_size, dtype=np.int16)

result = runner.classify(audio)
print(result)

runner.stop()
```

---

# Inference Workflows

## Image Classification

### Steps
1. Load model  
2. Load image  
3. Convert BGR → RGB  
4. Preprocess using Studio settings  
5. Run inference  
6. Interpret results  

### Example
```python
img = cv2.imread("cat.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

features, cropped = runner.get_features_from_image_auto_studio_settings(img)
res = runner.classify(features)

for label, score in res['result']['classification'].items():
    print(f"{label}: {score:.2f}")
```

---

## Object Detection

### Example
```python
res = runner.classify(features)

for bb in res['result']['bounding_boxes']:
    print(bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height'])
```

---

## Audio Classification

### Example
```python
audio_frame = np.frombuffer(stream.read(frame_size * 2), dtype=np.int16)
res = runner.classify(audio_frame)
```

---

## Continuous Inference Loop

### Example
```python
while True:
    img = camera.read()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    features, _ = runner.get_features_from_image_auto_studio_settings(img)
    res = runner.classify(features)

    print(res['result']['classification'])
```

---

# Best Practices

## Performance Tips
- Resize images before passing to the runner.  
- Convert features to NumPy arrays.  
- Reuse the same runner instance.  
- Always call `runner.stop()` when done.  

## Matching Studio Results
- Use `get_features_from_image_auto_studio_settings()`.  
- Ensure RGB input.  
- Use exact width/height from metadata.  
- Avoid extra normalization.  

## Debugging Preprocessing
```python
cv2.imwrite("debug_cropped.jpg", cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR))
```

---

# Troubleshooting

## “Features count mismatch”
Your feature vector length does not match:
```
model_info['model_parameters']['input_features_count']
```

## “Model failed to initialize”
Common causes:
- Missing `libedgeimpulse.so`  
- Wrong architecture  
- Corrupted `.eim` file  

## OpenCV issues
If `cv2.imread()` returns `None`, check:
- File path  
- Permissions  
- Image format  

---

# Appendix

## Classification Result Schema

```python
{
  "result": {
    "classification": { "label": float },
    "anomaly": float,
    "bounding_boxes": [
      {
        "label": str,
        "value": float,
        "x": int,
        "y": int,
        "width": int,
        "height": int
      }
    ]
  },
  "timing": {
    "dsp": int,
    "classification": int,
    "anomaly": int
  }
}
```

---

# End of Document

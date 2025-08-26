# 🛰️ IRS-Aided Attack Detection Framework

This repository contains the source code, simulation setup, and model implementation for a secure UAV communication framework that integrates **Intelligent Reflecting Surfaces (IRS)** and a **CNN-LSTM deep learning model** for **real-time jamming detection** in 5G mmWave networks.

The framework is evaluated using **ns-3 simulations** under various jammer and user mobility scenarios.

---

## 📌 Features

- 🔄 **IRS-Assisted Communication**: Enhances resilience by rerouting signals around obstacles and jammers.
- 🤖 **Deep Learning-Based Detection**: Classifies:
  - Normal communication
  - Jammed signals
  - IRS-Aided Jammed signals
- 🛰️ **UAV-mounted Base Station**: Simulated at 25 m altitude using mmWave links in LoS and NLoS environments.
- 📉 **Benchmarking**: Compared against traditional classifiers (Random Forest, XGBoost, CatBoost).
- 📊 **Performance**: Achieved **87.26%** validation accuracy with reduced false positives.

---

## 🛠️ Technologies Used

- `ns-3` (modified mmWave + IRS support)
- Python (`TensorFlow`, `Keras`, `NumPy`, `Matplotlib`)
- CNN-LSTM deep learning architecture
- Data formats: SINR and RSSI time-series logs (CSV)

---

## 📊 Results & Evaluation

The proposed **CNN-LSTM model** outperforms traditional ML classifiers in detecting jamming in IRS-assisted UAV communication networks.

**Model Validation Accuracy Comparison**

![Validation Accuracy](results/validation_accuracy.png)

| Model                 | Validation Accuracy |
| --------------------- | ------------------- |
| Random Forest         | 87.15%              |
| XGBoost               | 80.76%              |
| CatBoost              | 72.50%              |
| **Proposed CNN-LSTM** | **87.26%**          |

---

## ⚙️ Simulation Parameters

| Parameter               | Value                  |
| ----------------------- | ---------------------- |
| Terrestrial Users (UEs) | 5                      |
| Base Station (UAV)      | 1                      |
| UAV Jammer              | 1                      |
| UE Positions            | (0,0,1.5) → (40,0,1.5) |
| UAV Position            | (30, 10, 25)           |
| Jammer Position         | (15, 5, 10)            |
| IRS Position            | (15, 7.5, 15)          |
| IRS Gain                | 300 dB                 |
| IRS Rician K-factor     | 3.0                    |
| IRS Elements per UE     | 64                     |
| UE Transmit Power       | 10 dBm                 |
| Jammer Transmit Power   | 3000 dBm               |
| Simulation Duration     | 180 seconds            |

<!-- ---

## 🚀 Getting Started

```bash
# Clone this repository
git clone https://github.com/Vedantspit/IRS_Jamming_Sim_Detection.git

# Navigate into the project directory
cd IRS_Jamming_Sim_Detection

# Install dependencies
pip install -r requirements.txt

# Run ns-3 simulation
./waf --run irs_jamming_scenario

# Train the CNN-LSTM model
python train_model.py
``` -->

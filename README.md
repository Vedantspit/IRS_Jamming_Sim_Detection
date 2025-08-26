# 🛰️ IRS-Aided Attack Detection Framework

This repository contains the source code, simulation setup, and model implementation for a secure UAV communication framework that integrates **Intelligent Reflecting Surfaces (IRS)** and a **CNN-LSTM deep learning model** for **real-time jamming detection** in 5G mmWave networks.

The framework is evaluated using **ns-3 simulations** under various jammer and user mobility scenarios.

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
- Data formats: SINR and RSSI time-series logs (CSV) -->

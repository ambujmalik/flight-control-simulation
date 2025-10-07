# flight-control-simulation
Interactive flight simulation dashboard built with Python. Visualizes 3D aircraft motion and real-time control inputs using sliders for altitude, heading, and airspeed. Features dynamic Matplotlib plots, reset option, and ipywidgets-based controls for an engaging visualization experience.
# Flight Simulation Visualization Dashboard ✈️

An interactive Python-based tool for visualizing aircraft flight parameters such as **Altitude**, **Airspeed**, and **Heading**. It features real-time 3D trajectory plotting, control input tracking, and adjustable sliders to explore flight dynamics.

---

## 🚀 Features

* 3D flight path visualization (X, Y, Altitude)
* Real-time control input graphs (Elevator, Aileron, Rudder, Throttle)
* Dynamic flight parameter plots (Altitude, Airspeed, Angle of Attack)
* Adjustable sliders for:

  * Altitude (m)
  * Heading (°)
  * Airspeed (m/s)
* Reset button to restore initial state

---

## 🧠 Built With

* **Python 3**
* **Matplotlib** – for 3D plotting
* **ipywidgets** – for interactive controls
* **NumPy** – for simulation data

---

## 💻 Installation

```bash
git clone https://github.com/your-username/flight-dashboard.git
cd flight-dashboard
pip install matplotlib ipywidgets numpy
```

---

## ▶️ Usage

### In Jupyter Notebook:

```python
%matplotlib widget
```

Run the notebook and use sliders to adjust flight parameters.

### In Google Colab:

```python
from google.colab import output
output.enable_custom_widget_manager()
```

Then execute the notebook cells to view the interactive dashboard.

---

## ⚠️ Note

Matplotlib GUI sliders/buttons may not fully work in Colab due to backend limitations.
For full interactivity, run locally in **JupyterLab** or **VS Code Notebook** mode.

---

## 🧑‍💻 Author

**Ambuj Kishore Malik**
📅 Last Updated: October 2025

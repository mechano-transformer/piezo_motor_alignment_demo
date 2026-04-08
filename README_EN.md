# Autocollimator & Piezo Motor Alignment Demo

This application demonstrates **Automated Drift Correction (ADC)** by combining an **autocollimator** with a **piezo motor controller**.

Supported controllers:

* **PAMC-204** (via DLL, default)
* **PAMC-104** (direct RS232 communication using `--mode pamc104`)

---

## Demo Video

[Click here](https://s3.ap-northeast-1.wasabisys.com/marketingmaterial/Active%20Laser%20Beam%20Stabilization%20Systems2.mp4) to view the demo video.

---

# GUI Overview

![GUI Screenshot](window.png)

The GUI consists of three panels from left to right:

1. **Autocollimator**
2. **Piezo Motor Control**
3. **Automated Drift Correction (ADC)**

---

# Application Features

| Feature                              | Description                                                                                                                        |
| ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| **Autocollimator Data Acquisition**  | Continuously reads X/Y tilt angles from the autocollimator via serial communication and displays them in real time                 |
| **Manual Piezo Motor Control**       | Allows manual relative movement and stop control of the piezo motor via DLL or RS232                                               |
| **Automated Drift Correction (ADC)** | Automatically drives the piezo motor to minimize the autocollimator angular error using a gradient-descent-based control algorithm |
| **Position Routine**                 | Automatically moves through predefined target positions and waits for ADC convergence at each point                                |
| **Data Logging**                     | Records measurement data (timestamp, X/Y tilt angles) and saves it as CSV                                                          |

---

# ADC Operation Principle

```
Autocollimator → calculate angular error → drive piezo motors (ch1/ch2) → reduce error
```

Important implementation details:

* The controller **does not support simultaneous 2-axis driving**, so motion is performed sequentially (X → Y).
* X and Y errors are calculated **once at the start of the step** and not recomputed after X-axis movement.
* Errors below the convergence threshold are ignored to prevent overshoot.
* Calibration: **1000 pulses ≈ 2.74 angular units** (~365 pulses/unit).
* **Axis mapping (default):**

```
Axis 1 = Y-axis
Axis 2 = X-axis
Swap X/Y = OFF
```

---

# Project Structure

```
demo/
├── __init__.py
├── main.py
├── gui.py
├── pamc204_wrapper.py
├── pamc104_wrapper.py
├── ac_thread.py
├── adc_thread.py
└── position_routine.py
```

| File                  | Description                                    |
| --------------------- | ---------------------------------------------- |
| `main.py`             | Application entry point                        |
| `gui.py`              | Main GUI class (`ADCGUI`) and mode definitions |
| `pamc204_wrapper.py`  | PAMC-204 DLL wrapper                           |
| `pamc104_wrapper.py`  | PAMC-104 RS232 communication wrapper           |
| `ac_thread.py`        | Autocollimator acquisition thread              |
| `adc_thread.py`       | ADC control loop                               |
| `position_routine.py` | Automated position routine thread              |

---

# Requirements

**Operating System**

* Windows (required for `pamc204.dll`)

**Python**

* Python 3.10+

**Python dependencies**

```
pip install pyserial numpy
```

**PAMC-204 DLL**

If using PAMC-204 mode, place:

```
pamc204.dll
```

inside the `demo/` directory.

---

# Running the Application

```
cd demo
```

### Default (PAMC-204 DLL mode)

```
python main.py
```

### Specify DLL path

```
python main.py --dll ./pamc204.dll --mode pamc204
```

### PAMC-104 RS232 mode

```
python main.py --mode pamc104
```

### PAMC-104 with explicit COM port

```
python main.py --mode pamc104 --port COM3
```

---

# Modes

| Mode                | Controller | Wrapper              | Communication |
| ------------------- | ---------- | -------------------- | ------------- |
| `pamc204` (default) | PAMC-204   | `pamc204_wrapper.py` | DLL API       |
| `pamc104`           | PAMC-104   | `pamc104_wrapper.py` | Direct RS232  |

---

# Command Line Options

| Option   | Description                             | Example               |
| -------- | --------------------------------------- | --------------------- |
| `--mode` | Controller mode (`pamc204` / `pamc104`) | `--mode pamc104`      |
| `--dll`  | Path to `pamc204.dll`                   | `--dll ./pamc204.dll` |
| `--port` | Serial port for PAMC-104                | `--port COM3`         |

---

# PAMC-104 Command Specification

PAMC-104 communicates via **RS232 commands without address specification**.

| Command          | Description         | Example          | Response             |
| ---------------- | ------------------- | ---------------- | -------------------- |
| `CON`            | Communication check | `CON`            | `OK`                 |
| `INF`            | Firmware version    | `INF`            | `PAMC-104 Ver:0.8.0` |
| `NRffffnnnnA`    | Forward drive       | `NR15000010A`    | `OK → FIN`           |
| `RRffffnnnnA`    | Reverse drive       | `RR15000010A`    | `OK → FIN`           |
| `NRffffXnnnnnnA` | Forward extended    | `NR1500X000100A` | `OK → FIN`           |
| `S`              | Stop                | `S`              | `FIN nnnnn`          |

Parameters:

```
ffff : drive frequency (1–1500 Hz)
nnnn : pulse count (0000–9999, 0000 = continuous)
A-D  : axis selection (A=ch1, B=ch2, C=ch3, D=ch4)
```

Serial settings:

```
115200 baud
8 data bits
no parity
1 stop bit
no flow control
CR+LF delimiter
```

---

# Panel Operation Guide

## Autocollimator Panel

Handles connection, reading, and display of autocollimator data.

### Connection Procedure

1. Click **Refresh Ports**
2. Select a COM port
3. Choose measurement unit (`min`, `deg`, `mdeg`, `urad`)
4. Click **Start Reading**
5. Click **Stop Reading** to stop

---

### Data Smoothing

| Option           | Description                    |
| ---------------- | ------------------------------ |
| Enable Smoothing | Enables moving-average filter  |
| Averaging Window | Number of samples (default: 5) |

---

### Display Canvas

* **Red crosshair**: current autocollimator reading
* **Green crosshair**: ADC target position
* Numeric display of X/Y tilt values

---

## Piezo Motor Control Panel

Allows manual control of the piezo motor.

### PAMC-204 Connection

1. Enter device **Address** (default: `1`)
2. Click **Connect PAMC-204**
3. Status will indicate success or failure
4. Disconnect using **Disconnect**

Upon successful connection, **all channel speeds are automatically set to 1500 Hz**.

---

### Axis Configuration

| Option         | Description             |
| -------------- | ----------------------- |
| Reverse Axis 1 | Reverse pulse direction |
| Reverse Axis 2 | Reverse pulse direction |
| Swap X/Y Axes  | Swap axis assignment    |

Default mapping:

| Swap X/Y | X axis | Y axis |
| -------- | ------ | ------ |
| OFF      | Axis 2 | Axis 1 |
| ON       | Axis 1 | Axis 2 |

---

## Automated Drift Correction (ADC) Panel

Performs automatic drift correction.

### Operation

1. Start autocollimator reading
2. Connect the controller
3. Adjust parameters if needed
4. Click **Start ADC**
5. Stop using **Stop ADC**

---

# ADC Parameters

| Parameter             | Default    | Description             |
| --------------------- | ---------- | ----------------------- |
| Sample Period         | 0.5 s      | Control loop interval   |
| Learning Rate         | 0.3        | Correction gain         |
| Min Step              | 1 pulse    | Minimum correction step |
| Max Step              | 500 pulses | Maximum correction step |
| Piezo Calibration     | 365        | pulses per angle unit   |
| Convergence Threshold | 0.01       | error threshold         |

---

# Parameter Tuning Guide

| Symptom                  | Adjustment             |
| ------------------------ | ---------------------- |
| Slow convergence         | Increase learning rate |
| Oscillation              | Reduce learning rate   |
| Excessive micro-movement | Increase threshold     |
| Step too large           | Reduce max step        |

---

# Position Routine

Automatically visits five target positions:

| Step | Target Position | Hold Time |
| ---- | --------------- | --------- |
| 1    | (+7.0, −7.0)    | 2 s       |
| 2    | (−7.0, −7.0)    | 2 s       |
| 3    | (−7.0, +7.0)    | 2 s       |
| 4    | (+7.0, +7.0)    | 2 s       |
| 5    | (0.0, 0.0)      | return    |

ADC must converge **5 consecutive cycles** within the threshold before continuing.

Maximum wait time: **30 seconds**.

---

# Data Logging

Logging panel allows recording of measurement data.

1. Click **Start Test**
2. Click **Stop Test**
3. Click **Save Data to File**

Output format:

```
Time(s)    alnx        alny
0.000000   0.001234    -0.000567
0.100000   0.001189    -0.000512
```


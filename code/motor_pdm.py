# importing libraries
import time
import math
import socket
import subprocess
import numpy as np
import pandas as pd
import joblib
import signal
from smbus2 import SMBus
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont
from pathlib import Path
from runtime_baseline import RuntimeBaseline 


# OLED setup
OLED_ADDRESS = 0x3C
serial = i2c(port=1, address=OLED_ADDRESS)
oled = sh1106(serial)
font = ImageFont.load_default()

# sensor addresses
TEMPERATURE_SENSOR_ADDRESS = 0x5A
VIBRATION_SENSOR_ADDRESS = 0x53
CURRENT_SENSOR_ADDRESS = 0x48

# current sensor configuration
CONFIG = 0xC183
LSB = 6.144 / 32768.0
CALIBRATION_CONSTANT = 0.00437

# sensor status
sensor_status = {
    "temperature": False,
    "vibration": False,
    "current": False
}

current_filtered = None
CURRENT_OFFSET = None

# real-time global values
amb = None
obj = None
vib = None
cur = None
vx = 0
vy = 0
vz = 0
hi = 0.0
state = "N/A"
anomaly = "N/A"
anomaly_pct = 0
prediction_made = False

# NEW
long_press_count = 0   # tracks which stage after initialization

# timeout handling
class TimeoutError(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutError()

def safe_i2c_call(func, *args):
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1)

    try:
        result = func(*args)
        signal.alarm(0)
        return result
    except Exception as e:
        print("I2C ERROR:", e)
        return None
    finally:
        signal.alarm(0)

# WIFI
def get_wifi_name():
    try:
        ssid = subprocess.check_output(["iwgetid", "-r"], text=True).strip()
        return ssid if ssid else "Not connected"
    except:
        return "Unavailable"

def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "Unavailable"

# current sensor
def write_adc_config(bus):
    bus.write_i2c_block_data(CURRENT_SENSOR_ADDRESS, 0x01,
        [(CONFIG >> 8) & 0xFF, CONFIG & 0xFF])

def read_adc(bus):
    write_adc_config(bus)

    for _ in range(20):
        status = bus.read_i2c_block_data(CURRENT_SENSOR_ADDRESS, 0x01, 2)
        if status[0] & 0x80:
            break
    else:
        raise TimeoutError("ADC timeout")

    data = bus.read_i2c_block_data(CURRENT_SENSOR_ADDRESS, 0x00, 2)
    raw = (data[0] << 8) | data[1]

    if raw > 32767:
        raw -= 65536

    return raw * LSB


def adc_alive(bus):
    try:
        bus.read_i2c_block_data(CURRENT_SENSOR_ADDRESS, 0x01, 1)
        return True
    except OSError:
        return False

def measure_current(bus):
    global current_filtered

    if CURRENT_OFFSET is None:
        return 0, 0, False

    if not adc_alive(bus):
        sensor_status["current"] = False
        return 0, 0, False

    try:
        samples = []

        for _ in range(20):
            try:
                v = read_adc(bus)
                samples.append(v)

            except OSError:
                continue

        if len(samples) < 10:
            return 0, 0, False

        sum_sq = 0

        for v in samples:
            ac = v - CURRENT_OFFSET
            sum_sq += ac * ac

        vrms = math.sqrt(sum_sq / len(samples))

        current = vrms / CALIBRATION_CONSTANT

        if current < 0.15:
            current = 0

        if current_filtered is None:
            current_filtered = current
        else:
            alpha = 0.2
            current_filtered = (alpha * current + (1 - alpha) * current_filtered)
        
        sensor_status["current"] = True
        return current_filtered, vrms, True

    except Exception as e:
        print("CURRENT SENSOR ERROR:", e)
        return 0, 0, False

# temperature sensor
def read_temp_safe(bus, reg):
    try:
        raw = bus.read_word_data(TEMPERATURE_SENSOR_ADDRESS, reg)
        sensor_status["temperature"] = True
        raw = raw & 0x7FFF # mask out error flag bit 15
        return (raw * 0.02) - 273.15
    except:
        sensor_status["temperature"] = False
        return None

def read_temperature(bus):
    amb = read_temp_safe(bus, 0x06)
    obj = read_temp_safe(bus, 0x07)
    return amb, obj


# vibration sensor
def init_adxl345(bus):
    try:
        bus.write_byte_data(VIBRATION_SENSOR_ADDRESS, 0x2D, 0x08)
        bus.write_byte_data(VIBRATION_SENSOR_ADDRESS, 0x31, 0x08)
        bus.write_byte_data(VIBRATION_SENSOR_ADDRESS, 0x2C, 0x0A)
        print("ADXL345 initialized")
    except Exception as e:
        print("ADXL345 INIT ERROR:", e)

def read_vibration(bus):
    try:
        data = bus.read_i2c_block_data(VIBRATION_SENSOR_ADDRESS, 0x32, 6)
        sensor_status["vibration"] = True

        x = data[0] | (data[1] << 8)
        y = data[2] | (data[3] << 8)
        z = data[4] | (data[5] << 8)

        if x > 32767: x -= 65536
        if y > 32767: y -= 65536
        if z > 32767: z -= 65536

        x *= 0.0039
        y *= 0.0039
        z *= 0.0039
        z_corrected = z - 1.0 # remove gravity component (assumes Z is vertical)

        vib = np.sqrt(x**2 + y**2 + z_corrected**2)
        return vib, x, y, z

    except:
        sensor_status["vibration"] = False
        return 0, 0, 0, 0


def update_sensors(bus):
    global amb, obj, vib, cur, vx, vy, vz

    amb_new, obj_new = safe_i2c_call(read_temperature, bus) or (None, None)
    vib_new = safe_i2c_call(read_vibration, bus) or (0, 0, 0, 0)
    cur_new, _, _ = measure_current(bus)

    if amb_new is not None:
        amb = amb_new
    if obj_new is not None:
        obj = obj_new

    vib, vx, vy, vz = vib_new
    cur = cur_new

def update_sensor_status(bus):
    
    # temperature
    try:
        amb, obj = read_temperature(bus)
        sensor_status["temperature"] = (amb is not None and obj is not None)
    except:
        sensor_status["temperature"] = False

    # vibration
    try:
        vib, x, y, z = read_vibration(bus)
        sensor_status["vibration"] = (vib != 0)
    except:
        sensor_status["vibration"] = False

    # current
    sensor_status["current"] = adc_alive(bus)

ML_READY = True

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = PROJECT_ROOT / "models" / "pkl"

try:
    state_model          = joblib.load(MODEL_DIR / "statemodel.pkl")
    anomaly_model        = joblib.load(MODEL_DIR / "anomalymodel.pkl")
    hi_model             = joblib.load(MODEL_DIR / "himodel.pkl")
    preprocessor_state   = joblib.load(MODEL_DIR / "statepreprocessor.pkl")
    preprocessor_anomaly = joblib.load(MODEL_DIR / "anomalypreprocessor.pkl")
    preprocessor_hi      = joblib.load(MODEL_DIR / "hipreprocessor.pkl")
    state_enc             = joblib.load(MODEL_DIR / "state_encoder.pkl")
    anomaly_enc           = joblib.load(MODEL_DIR / "anomaly_encoder.pkl")
    training_baselines    = joblib.load(MODEL_DIR / "training_baselines.pkl")
    model_features       = joblib.load(MODEL_DIR / "model_features.pkl")

except Exception as e:
    print("MODEL LOAD ERROR:", e)
    ML_READY = False


baseline = RuntimeBaseline(seed=training_baselines if ML_READY else None)

# prediction
def run_inference(irms_raw: float, vib_rms_raw: float, temp_rise_raw: float):

    ratio_features = baseline.compute_ratios(irms_raw, vib_rms_raw, temp_rise_raw)

    if ratio_features is None:
        return {'state': 'Initializing', 'anomaly': 'N/A', 'anomaly_pct': 0, 'health_index': None}

    row = pd.DataFrame([{
        'Irms_ratio': ratio_features['Irms_ratio'],
        'Vib_ratio':  ratio_features['Vib_ratio'],
        'Temp_ratio': ratio_features['Temp_ratio'],
    }])

    # State
    X_state     = preprocessor_state.transform(row)
    state_label = state_enc.inverse_transform(state_model.predict(X_state))[0]
# HI
    X_hi         = preprocessor_hi.transform(row)
    health_index = float(hi_model.predict(X_hi)[0])

    # Anomaly
    anomaly_type = 'N/A'
    anomaly_pct  = 0

    if state_label == 'Faulty':
        X_anomaly    = preprocessor_anomaly.transform(row)
        proba        = anomaly_model.predict_proba(X_anomaly)[0]
        top_idx      = proba.argmax()
        anomaly_type = anomaly_enc.classes_[top_idx]
        anomaly_pct  = int(proba[top_idx] * 100)

    return {
        'state':        state_label,
        'anomaly':      anomaly_type,
        'anomaly_pct':  anomaly_pct,
        'health_index': health_index,
    }

def center(draw, text, y):
    x = (oled.width - draw.textlength(text, font=font)) // 2
    draw.text((x, y), text, font=font, fill=255)


# button set
import RPi.GPIO as GPIO

BUTTON_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

last_button_state = 1
button_press_time = None

mode = "idle"
initialization_progress = 0
initialization_done = False

def wifi_page():
    with canvas(oled) as draw:
        center(draw, "WIFI STATUS", 0)
        center(draw, get_wifi_name(), 22)
        center(draw, get_ip_address(), 44)

def sensor_page(bus):
    update_sensor_status(bus)
    temp_status = ("CONNECTED"if sensor_status["temperature"] else "NOT CONNECTED")
    vib_status = ("CONNECTED" if sensor_status["vibration"] else "NOT CONNECTED")
    cur_status = ("CONNECTED"if sensor_status["current"] else "NOT CONNECTED")

    with canvas(oled) as draw:
        center(draw, "SENSOR STATUS", 0)
        draw.text((0, 18), f"TEMP : {temp_status}", font=font, fill=255)
        draw.text((0, 34), f"VIB  : {vib_status}", font=font, fill=255)
        draw.text((0, 50), f"CURR : {cur_status}", font=font, fill=255)

def temp_page(bus):
    status = "(Sensor Connected)" if sensor_status["temperature"] else "(Sensor not Connected)"
    with canvas(oled) as draw:
        center(draw, "TEMPERATURE STATUS", 0)
        if sensor_status["temperature"] and amb is not None:
            center(draw, status, 20)
            center(draw, f"Ambient: {amb:.2f} C", 38)
            center(draw, f"Object: {obj:.2f} C", 52)
        else:
            center(draw, "(Sensor not Connected)", 40)
    return amb, obj
    
def vibration_page(bus):
    status = "(Sensor Connected)" if sensor_status["vibration"] else "(Sensor not Connected)"
    with canvas(oled) as draw:
        center(draw, "VIBRATION STATUS", 0)
        center(draw, status, 20)
        center(draw, f"X:{vx:.3f} Y:{vy:.3f} Z:{vz:.3f}", 38)
        center(draw, f"Magnitude: {vib:.4f}", 52)
    return vib

def current_page(bus):
    status = "(Sensor Connected)" if sensor_status["current"] else "(Sensor not Connected)"
    with canvas(oled) as draw:
        center(draw, "CURRENT STATUS", 0)
        center(draw, status, 20)
        center(draw, f"Current RMS: {cur:.4f} A", 52)
    return cur

def calibrating_page():
    with canvas(oled) as draw:
        center(draw, "MOTOR STATUS", 0)
        center(draw, "Hold button >3s", 18)
        center(draw, "to determine", 32)
        center(draw, "motor status", 46)

def status_page_1(state, hi):
    with canvas(oled) as draw:
        center(draw, "MOTOR STATUS", 0)
        center(draw, f"State: {state}", 22)
        center(draw, f"HI: {hi:.4f}", 42)  

def status_page_2(anomaly, anomaly_pct):
    with canvas(oled) as draw:
        center(draw, "FAULT DETAIL", 0)
        if anomaly == 'N/A':
            center(draw, "No fault", 28)
        else:
            center(draw, anomaly[:16], 20)
            center(draw, anomaly[16:], 32)
            center(draw, f"Likelihood: {anomaly_pct}%", 48)
      
def monitoring_page(progress):
    with canvas(oled) as draw:
        center(draw, "MOTOR STATUS", 0)
        center(draw, "Collecting data...", 16)
        center(draw, f"{progress}%", 34)
        bar_width = int((oled.width - 20) * progress / 100)
        draw.rectangle((10, 45, 10 + bar_width, 58), fill=255)

def idle_page():
    with canvas(oled) as draw:
        center(draw, "PdM SYSTEM", 0)
        center(draw, "--Mount sensors--", 16)
        center(draw, "--Remove current clamp--", 28)
        center(draw, "--Hold button >3 secs to", 42)
        center(draw, "initialize system--", 52)

def full_reset():
    global baseline, long_press_count
    global state, anomaly, anomaly_pct, hi, prediction_made
    global initialization_done, CURRENT_OFFSET, current_filtered
    baseline            = RuntimeBaseline(seed=training_baselines if ML_READY else None)
    long_press_count    = 0
    state               = "N/A"
    anomaly             = "N/A"
    anomaly_pct         = 0
    hi                  = 0.0
    prediction_made     = False
    initialization_done = False
    CURRENT_OFFSET      = None
    current_filtered    = None
    print("\n=== SYSTEM RESET back to pre-initialization ===\n")
    with canvas(oled) as draw:
        center(draw, "SYSTEM RESET", 16)
        center(draw, "Hold 3s to init", 38)
    time.sleep(2)

def main():
    global last_button_state, button_press_time
    global initialization_done, CURRENT_OFFSET
    global state, anomaly, anomaly_pct, hi, prediction_made, long_press_count
    mode = "idle"
    initialization_progress = 0
    initialization_index = 0
    initialization_samples = 100
    monitoring_index = 0
    monitoring_data  = []
    pages = ["wifi", "temp", "vibration", "current", "status1", "status2"]
    page_index = 0
    with SMBus(1) as bus:
        init_adxl345(bus)
        while True:
            # button logic
            current_state = GPIO.input(BUTTON_PIN)
            
            # detect press start
            if last_button_state == 1 and current_state == 0:
                button_press_time = time.time()

            # detect release
            if last_button_state == 0 and current_state == 1:
                if button_press_time is not None:
                    held = time.time() - button_press_time

                    if held >= 10:
                        # ULTRA LONG PRESS - full system reset
                        full_reset()
                        mode = "idle"

                    elif held >= 3:
                        if not initialization_done:
                            # PRESS 1 - offset calibration
                            mode = "initializing"
                            initialization_index = 0

                        elif long_press_count == 0:
                            # PRESS 2 - predict using training baseline
                            long_press_count = 1
                            mode = "monitoring"     #  re-run prediction with new baseline
                            monitoring_index = 0
                            monitoring_data  = []
                          
                        elif long_press_count == 1:
                            # PRESS 3 - enter known healthy values via terminal
                            long_press_count = 2
                            mode = "baseline_input"

                        elif long_press_count == 2:
                            # PRESS 4+ - re-enter baseline input (redo press 3)
                            mode = "baseline_input"
                    else:
                        # SHORT PRESS
                        if not initialization_done:
                            if mode == "idle":
                                mode = "sensor_check"
                            elif mode == "sensor_check":
                                mode = "wifi_only"
                            else:
                                mode = "idle"
                        else:
                            if mode == "baseline_input":
                                # dismiss terminal input, go back to run
                                mode = "run"
                                print("\n[Baseline input dismissed using previous baseline]\n")
                            elif mode == "idle":
                                mode = "run"
                            elif mode == "run":
                                page_index = (page_index + 1) % len(pages)

                button_press_time = None
                time.sleep(0.05)

            last_button_state = current_state
             # System initialization
            if mode == "initializing":
                samples = []
                while initialization_index < initialization_samples:
                    try:
                        v = read_adc(bus)
                        samples.append(v)
                    except:
                        pass
                    initialization_index += 1
                    initialization_progress = int((initialization_index / initialization_samples) * 100)

                    with canvas(oled) as draw:
                        center(draw, "System Initialization", 0)
                        center(draw, "Please Wait...", 16)
                        center(draw, f"{initialization_progress}%", 34)

                        bar_width = int((oled.width - 20) * initialization_progress / 100)
                        draw.rectangle((10, 45, 10 + bar_width, 58), fill=255)
                else:
                    if len(samples) > 0:
                        CURRENT_OFFSET = np.median(samples)
                    else:
                        CURRENT_OFFSET = 1.65  # fallback safe value

                    print("CURRENT OFFSET SET:", CURRENT_OFFSET)

                    initialization_done = True
                    page_index = 1
                    mode = "run"
                    initialization_index = 0
                    initialization_progress = 0

                    with canvas(oled) as draw:
                        center(draw, "Initialization Complete", 20)
                        center(draw, "System Ready", 40)

                    time.sleep(2)

                continue   
                # Motor monitoring data collection
                 # Collect samples, build baseline, take single prediction, lock it
            if mode == "monitoring":
                update_sensors(bus)

                COLLECT_SAMPLES = 60
                monitoring_progress = int((monitoring_index / COLLECT_SAMPLES) * 100)
                monitoring_page(monitoring_progress)

                if CURRENT_OFFSET is not None and amb is not None and obj is not None and cur is not None and vib is not None:
                    monitoring_data.append({
                        'cur': cur,
                        'vib': vib,
                        'temp_rise': obj - amb
                    })
                    monitoring_index += 1
            
                if monitoring_index >= COLLECT_SAMPLES:
                    if ML_READY and len(monitoring_data) > 0:
                        try:
                            mean_cur       = np.mean([s['cur']       for s in monitoring_data])
                            mean_vib       = np.mean([s['vib']       for s in monitoring_data])
                            mean_temp_rise = np.mean([s['temp_rise'] for s in monitoring_data])

                            result      = run_inference(mean_cur, mean_vib, mean_temp_rise)
                            state       = result['state']
                            anomaly     = result['anomaly']
                            anomaly_pct = result['anomaly_pct']
                            hi          = result['health_index'] if result['health_index'] is not None else 0.0
                            prediction_made = True
                        except Exception as e:
                            print("PREDICTION ERROR:", e)

                    monitoring_index = 0
                    monitoring_data  = []
                    mode = "run"
                    page_index = pages.index("status1")

                    with canvas(oled) as draw:
                        center(draw, "Status Ready", 30)
                    time.sleep(2)

                time.sleep(0.05)
                continue
            if mode == "baseline_input":
                INPUT_FILE = PROJECT_ROOT / "baseline_input.txt"
                
                instruction_pages = [
                    ["BASELINE INPUT", "", "Write a file named", "baseline_input.txt"],
                    ["LINE ORDER:", "1: Ambient Temp C", "2: Object Temp  C", "3: Current RMS A"],
                    ["LINE ORDER:", "4: Vibration X  g", "5: Vibration Y  g", "6: Vibration Z  g"],
                    ["EXAMPLE:", "1: 22.50", "2: 51.60", "3: 0.58"],
                    ["EXAMPLE:", "4: 3.65", "5: -1.12", "6: -7.15"],
                    ["THEN SAVE FILE", "Service will read", "it automatically", "and delete it."],
                ]
                instr_index = 0
                instr_timer = time.time()

                print("\nWaiting for baseline_input.txt ...")
                print("File format (one value per line):")
                print("  Line 1: Ambient Temp (C)")
                print("  Line 2: Object Temp  (C)")
                print("  Line 3: Current RMS  (A)")
                print("  Line 4: Vibration X  (g)")
                print("  Line 5: Vibration Y  (g)")
                print("  Line 6: Vibration Z  (g)")
                print("\nShort press button to cancel.\n")
                # wait for file or button cancel
                while True:
                    # cycle instruction pages every 3 seconds
                    if time.time() - instr_timer > 3:
                        instr_index = (instr_index + 1) % len(instruction_pages)
                        instr_timer = time.time()

                    lines = instruction_pages[instr_index]
                    with canvas(oled) as draw:
                        y_positions = [0, 14, 28, 42]
                        for i, line in enumerate(lines):
                            center(draw, line, y_positions[i])

                    # check button for cancel
                    current_state = GPIO.input(BUTTON_PIN)
                    if last_button_state == 0 and current_state == 1:
                        held = time.time() - button_press_time if button_press_time else 0
                        if held < 3:
                            print("[Baseline input cancelled]\n")
                            with canvas(oled) as draw:
                                center(draw, "CANCELLED", 24)
                                center(draw, "Using prev baseline", 40)
                            time.sleep(2)
                            mode = "run"
                            break
                    last_button_state = current_state

                    import os
                    if os.path.exists(INPUT_FILE):
                        try:
                            with open(INPUT_FILE, 'r') as f:
                                lines = [l.strip() for l in f.readlines() if l.strip()]

                            if len(lines) < 6:
                                raise ValueError("Need 6 values")

                            amb_input = float(lines[0])
                            obj_input = float(lines[1])
                            cur_input = float(lines[2])
                            vx_input  = float(lines[3])
                            vy_input  = float(lines[4])
                            vz_input  = float(lines[5])

                            temp_rise_input = obj_input - amb_input
                            vz_corrected    = vz_input - 1.0
                            vib_input       = math.sqrt(vx_input**2 + vy_input**2 + vz_corrected**2)

                            baseline.force_set(cur_input, vib_input, temp_rise_input)

                            print(f"\n  Baseline set:")
                            print(f"    Irms      = {cur_input:.4f} A")
                            print(f"    Vib mag   = {vib_input:.4f} g")
                            print(f"    Temp rise = {temp_rise_input:.4f} C\n")

                            os.remove(INPUT_FILE)   # delete so it doesn't re-trigger

                            with canvas(oled) as draw:
                                center(draw, "BASELINE SET", 0)
                                center(draw, f"I:  {cur_input:.4f} A", 16)
                                center(draw, f"V:  {vib_input:.4f} g", 32)
                                center(draw, f"dT: {temp_rise_input:.4f} C", 48)
                            time.sleep(3)

                            mode = "monitoring"
                            monitoring_index = 0
                            monitoring_data  = []
                            break

                        except (ValueError, IndexError) as e:
                            print(f"  Bad file: {e} fix and re-save.\n")
                            os.remove(INPUT_FILE)
                            with canvas(oled) as draw:
                                center(draw, "BAD FILE", 16)
                                center(draw, "Check values", 32)
                                center(draw, "and re-save", 48)
                            time.sleep(3)

                    time.sleep(0.2)

                continue
            # page control
            if mode == "idle":
                idle_page()
                time.sleep(0.2)
                continue
            if mode == "sensor_check":
                sensor_page(bus)
                time.sleep(0.2)
                continue

            if mode == "wifi_only":
                wifi_page()
                time.sleep(0.2)
                continue
                
            elif mode == "run":
                update_sensors(bus)
                
                if pages[page_index] == "wifi":
                    wifi_page()

                elif pages[page_index] == "temp":
                    amb, obj = temp_page(bus)

                elif pages[page_index] == "vibration":
                    vib = vibration_page(bus)

                elif pages[page_index] == "current":
                    cur = current_page(bus)

                elif pages[page_index] == "status1":
                    if not prediction_made:
                        calibrating_page()
                    else:
                        status_page_1(state, hi)  

                elif pages[page_index] == "status2":
                    if prediction_made:
                        status_page_2(anomaly, anomaly_pct)
                    else:
                        calibrating_page()
                time.sleep(0.05)
if __name__ == "__main__":
    main()


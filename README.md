# -*- coding: utf-8 -*-
import serial, time, threading, glob
from RPi import GPIO

# ===== 핀 배치 =====
# 서보 (MG995) 신호
SERVO1 = 12   # 서보1 (좌/우)
SERVO2 = 13   # 서보2 (Z축)

# L298N 스텝모터 (리니어)
IN1, IN2, IN3, IN4 = 17, 18, 27, 22

# 토글 입력/출력 (버튼 ↔ GND, INPUT_PULLUP)
TRIG_IN  = 24   # GPIO24: 버튼 누를 때마다 토글
OUT_SIG  = 23   # GPIO23: 토글 상태에 따라 HIGH(ON)/LOW(OFF)

# 긴급 종료 입력 (버튼 ↔ GND, INPUT_PULLUP)
E_STOP_IN = 25  # GPIO25: 버튼 누르면 즉시 안전 종료

# ===== 파라미터 =====
STEP_DEG   = 20
MIN_ANGLE  = 0
MAX_ANGLE  = 180
SETTLE_S   = 0.35

STEP_DELAY = 0.003  # 스텝모터 속도(작을수록 빠름)
SEQ = [
    (1,0,0,0),(1,0,1,0),(0,0,1,0),(0,1,1,0),
    (0,1,0,0),(0,1,0,1),(0,0,0,1),(1,0,0,1)
]

# 시리얼/스텝 상태
ser = None
angle1, angle2 = 90, 90
step_running = False
step_direction = 0
last_status_at = 0.0
STATUS_TIMEOUT = 0.25  # STATUS 못 받으면 정지

# 버튼 디바운스 & 상태 (풀업이라 기본 HIGH=1)
EDGE_DEBOUNCE_S = 0.20
last_in24_state = 1
last_in24_ts    = 0.0
out23_on        = False  # True면 GPIO23을 계속 HIGH 유지

# 긴급 종료(25) 디바운스 & 상태 (풀업이라 기본 HIGH=1)
E_STOP_DEBOUNCE_S = 0.08
last_estop_state  = 1
last_estop_ts     = 0.0
terminate         = False  # True가 되면 메인루프 종료

# ===== 유틸 =====
def angle_to_duty(angle): return 2.5 + (angle / 18.0)
def clamp(x, lo, hi): return max(lo, min(hi, x))

# ===== 하드웨어 초기화 =====
def setup_hardware():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # 스텝모터 출력
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

    # 서보 PWM (50Hz)
    GPIO.setup(SERVO1, GPIO.OUT)
    GPIO.setup(SERVO2, GPIO.OUT)
    p1 = GPIO.PWM(SERVO1, 50)
    p2 = GPIO.PWM(SERVO2, 50)
    p1.start(0); p2.start(0)

    # 토글 I/O (풀업 → 기본 HIGH, 누르면 LOW)
    GPIO.setup(TRIG_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(OUT_SIG, GPIO.OUT, initial=GPIO.LOW)

    # 긴급 종료 입력 (풀업 → 기본 HIGH, 누르면 LOW)
    GPIO.setup(E_STOP_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    return p1, p2

# ===== 스텝모터 =====
def drive(a1,a2,b1,b2):
    GPIO.output(IN1,a1); GPIO.output(IN2,a2)
    GPIO.output(IN3,b1); GPIO.output(IN4,b2)

def step_once(direction):
    seq = SEQ if direction > 0 else reversed(SEQ)
    for s in seq:
        drive(*s)
        time.sleep(STEP_DELAY)

def step_motor_thread():
    global step_running, step_direction, terminate
    while True:
        if terminate:
            drive(0,0,0,0)
            return
        if step_running and step_direction != 0:
            step_once(step_direction)
        else:
            drive(0,0,0,0)
            time.sleep(0.01)

# ===== 서보 =====
def move_servo_to(pwm, angle):
    pwm.ChangeDutyCycle(angle_to_duty(angle))
    time.sleep(SETTLE_S)  # 이동 시간만 대기(브레이크 유지)

# ===== ESP32 포트 찾기 =====
def find_esp32():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    print("검색된 포트:", ports)
    for port in ports:
        try:
            s = serial.Serial(port, 115200, timeout=1)
            time.sleep(0.3)
            s.reset_input_buffer(); s.reset_output_buffer()
            s.write(b'HOST_READY\n')
            start = time.time()
            buf = b''
            while time.time() - start < 3.0:
                if s.in_waiting:
                    buf += s.read(s.in_waiting)
                    txt = buf.decode(errors='ignore')
                    if 'ESP32_READY' in txt or 'BTN_' in txt:
                        print("✅ ESP32:", port)
                        return s
                time.sleep(0.05)
            return s
        except:
            continue
    return None

# ===== 명령 처리(방향 반전: 서보만 반전, 리니어는 그대로) =====
def process(cmd, p1, p2):
    global angle1, angle2, step_running, step_direction, last_status_at
    if cmd.startswith("BTN_PRESS:"):
        btn = cmd.split(":")[1]

        # 서보 방향 반전(요청 반영)
        if btn == "LEFT":
            angle1 = clamp(angle1 + STEP_DEG, MIN_ANGLE, MAX_ANGLE)
            move_servo_to(p1, angle1); print("서보1:", angle1)

        elif btn == "RIGHT":
            angle1 = clamp(angle1 - STEP_DEG, MIN_ANGLE, MAX_ANGLE)
            move_servo_to(p1, angle1); print("서보1:", angle1)

        elif btn == "SERVO2_UP":
            angle2 = clamp(angle2 - STEP_DEG, MIN_ANGLE, MAX_ANGLE)
            move_servo_to(p2, angle2); print("서보2:", angle2)

        elif btn == "SERVO2_DOWN":
            angle2 = clamp(angle2 + STEP_DEG, MIN_ANGLE, MAX_ANGLE)
            move_servo_to(p2, angle2); print("서보2:", angle2)

    elif cmd.startswith("STATUS:"):
        last_status_at = time.time()
        # 리니어(Z)는 기존 방향 그대로
        if "Z_UP" in cmd:
            step_running, step_direction = True, +1
        elif "Z_DOWN" in cmd:
            step_running, step_direction = True, -1
        else:
            step_running, step_direction = False, 0

# ===== GPIO24 토글 감지 → GPIO23 유지 출력 =====
def poll_toggle_input():
    """풀업 구성: 버튼을 누르면 LOW(0). LOW 에지(HIGH→LOW)에서 토글"""
    global last_in24_state, last_in24_ts, out23_on
    now = time.time()
    cur = GPIO.input(TRIG_IN)  # 1=안 눌림, 0=눌림
    # FALLING 에지(1→0) + 디바운스
    if cur == 0 and last_in24_state == 1 and (now - last_in24_ts) > EDGE_DEBOUNCE_S:
        out23_on = not out23_on
        GPIO.output(OUT_SIG, GPIO.HIGH if out23_on else GPIO.LOW)
        print(f"[토글] GPIO24 버튼 → GPIO23 = {'HIGH(ON)' if out23_on else 'LOW(OFF)'}")
        last_in24_ts = now
    last_in24_state = cur

# ===== GPIO25 긴급 종료 감지 =====
def poll_emergency_stop(p1=None, p2=None):
    """풀업 구성: 버튼을 누르면 LOW(0). LOW 에지(1→0)에서 즉시 종료"""
    global last_estop_state, last_estop_ts, terminate
    now = time.time()
    cur = GPIO.input(E_STOP_IN)  # 1=안 눌림, 0=눌림
    if cur == 0 and last_estop_state == 1 and (now - last_estop_ts) > E_STOP_DEBOUNCE_S:
        print("⚠️ [E-STOP] GPIO25 버튼 감지 → 즉시 안전 종료")
        # 즉시 출력들 안전 상태로
        try:
            drive(0,0,0,0)
            GPIO.output(OUT_SIG, GPIO.LOW)
            if p1: p1.ChangeDutyCycle(0)
            if p2: p2.ChangeDutyCycle(0)
        except: pass
        terminate = True
        last_estop_ts = now
    last_estop_state = cur

# ===== 메인 =====
def main():
    global ser, last_status_at, terminate
    print("ESP32 연결 시도...")
    ser = find_esp32()
    if ser is None:
        print("ESP32 없음"); return

    p1, p2 = setup_hardware()
    move_servo_to(p1, angle1); move_servo_to(p2, angle2)

    threading.Thread(target=step_motor_thread, daemon=True).start()
    last_status_at = time.time()
    print("준비 완료 (Ctrl+C 종료)")

    try:
        while True:
            if terminate:
                break

            # 1) ESP32에서 메시지 수신
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    process(line, p1, p2)

            # 2) STATUS 타임아웃 → 리니어 정지
            if time.time() - last_status_at > STATUS_TIMEOUT:
                step_running = False
                step_direction = 0

            # 3) GPIO24 토글 버튼
            poll_toggle_input()

            # 4) GPIO25 긴급 종료 버튼
            poll_emergency_stop(p1, p2)

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        # 정리
        drive(0,0,0,0)
        GPIO.output(OUT_SIG, GPIO.LOW)
        try:
            p1.stop(); p2.stop()
        except:
            pass
        GPIO.cleanup()
        if ser:
            try: ser.close()
            except: pass
        print("정리 완료")

if __name__ == "__main__":
    main()

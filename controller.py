import socket
import json

HOST = "127.0.0.1"
PORT = 5000

# パラメータ調整用
V_MAX = 10.0			# 最高目標速度[m/s]
K_FRONT = 0.8			# front距離→速度スケール
CURVE_GAIN = 4.5  		# カーブ強度による減速係数
THROTTLE_SCALE = 0.1 	# 目標速度→スロットル変換用
STEER_GAIN = 0.2		# steering用ゲイン
SLOW_DIST = 30			# brakeをかける距離の閾値
STOP_DIST = 5
MIN_THROTTLE = 0.02

lastSteer = 0.0

def smooth(x, prev, alpha = 0.8):
	return prev + alpha * (x - prev)

def control_logic(state: dict) -> dict:
	"""
	state: Unityから来るJSONのdict
	     - state["lidar"]: 距離配列 (左→右)
	return:
	  - {"throttle": float, "steering": float}
	"""
    
	lidar = state["lidar"]
	n = len(lidar)

	if n < 5:
		return {"throttle": 0.0, "steering": 0.0}

	# ----- 1. 前方距離 -----
	center_indices = range(n//2 - 2, n//2 + 3)
	front = min(lidar[i] for i in center_indices)

	# ----- 2. 左右の空き具合 -----
	left_indices = range(0, n//3)
	right_indices = range(2*n//3, n)

	left_open = sum(lidar[i] for i in left_indices) / len(list(left_indices))
	right_open = sum(lidar[i] for i in right_indices) / len(list(right_indices))

	denom = left_open + right_open + 1e-6
	curve = abs(left_open - right_open) / denom  # カーブ強度(0~1)

    # ----- 3. 目標速度 v_target -----
	v_base = min(front * K_FRONT, V_MAX)
	v_target = v_base * (1.0 - CURVE_GAIN * curve)
	v_target = max(0.0, v_target)

	# ----- 4. throttle（v_target をそのままスケール） -----
	if front < STOP_DIST:
		throttle = 0
	elif front < SLOW_DIST:
		ratio = (front - STOP_DIST) / (SLOW_DIST - STOP_DIST)
		throttle = MIN_THROTTLE + ratio * (v_target * THROTTLE_SCALE - MIN_THROTTLE)
	else:
		throttle = v_target * THROTTLE_SCALE
		throttle = max(-1.0, min(1.0, throttle))

    # ----- 5. steering -----
	left_sample = lidar[int(n * 0.25)]
	right_sample = lidar[int(n * 0.75)]

	global lastSteer

	steer_raw = (right_sample - left_sample) * STEER_GAIN
	steer_raw = max(-1.0, min(1.0, steer_raw))
	steering = smooth(steer_raw, lastSteer)
	lastSteer = steering

	return {
        "throttle": throttle,
        "steering": steering,
    }


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[Python] Listening on {HOST}:{PORT}")

        conn, addr = s.accept()
        print("[Python] Connected:", addr)

        buf = b""
        with conn:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line:
                        continue

                    try:
                        state = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError:
                        continue

                    cmd = control_logic(state)
                    out = (json.dumps(cmd) + "\n").encode("utf-8")
                    conn.sendall(out)


if __name__ == "__main__":
    main()

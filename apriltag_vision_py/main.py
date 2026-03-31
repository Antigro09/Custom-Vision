import time
import cv2
import numpy as np

from config import load_config, load_calibration
from camera import open_camera
from detector import create_detector
from pose import estimate_pose
from publisher import Publisher

def cuda_available():
    try:
        return cv2.cuda.getCudaEnabledDeviceCount() > 0
    except Exception:
        return False

def main():
    cfg = load_config("config.json")
    K_list, D_list = load_calibration("calibration.json")
    K = np.array(K_list, dtype=np.float64)
    D = np.array(D_list, dtype=np.float64).reshape(-1, 1)

    print(f"[INFO] CUDA available: {cuda_available()}")

    cap = open_camera(cfg.camera_index, cfg.width, cfg.height, cfg.fps)
    detector = create_detector(cfg.tag_family)
    publisher = Publisher(cfg.publish_nt, cfg.team_number, cfg.nt_table)

    frame_id = 0
    prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        start = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        out = []
        for d in detections:
            if d.decision_margin < cfg.min_decision_margin:
                continue

            pose = estimate_pose(d.corners, K, D, cfg.tag_size_m)
            if pose is None:
                continue

            rvec, tvec, euler, dist = pose
            out.append({
                "id": int(d.tag_id),
                "decision_margin": float(d.decision_margin),
                "center": [float(d.center[0]), float(d.center[1])],
                "corners": np.array(d.corners, dtype=float).reshape(-1).tolist(),
                "tvec_m": tvec.reshape(-1).tolist(),
                "rvec_rad": rvec.reshape(-1).tolist(),
                "euler_deg": [float(euler[0]), float(euler[1]), float(euler[2])],
                "distance_m": dist
            })

            if cfg.draw_debug:
                c = np.array(d.corners, dtype=int)
                for i in range(4):
                    cv2.line(frame, tuple(c[i]), tuple(c[(i+1) % 4]), (0, 255, 0), 2)
                cv2.putText(frame, f"ID {d.tag_id}", tuple(c[0]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        now = time.time()
        fps = 1.0 / max(now - prev, 1e-6)
        prev = now
        latency_ms = (now - start) * 1000.0

        payload = {
            "connected": True,
            "frame_id": frame_id,
            "timestamp_us": int(now * 1_000_000),
            "latency_ms": latency_ms,
            "fps": fps,
            "tag_count": len(out),
            "detections": out
        }

        publisher.publish(payload)

        if cfg.draw_debug:
            cv2.putText(frame, f"FPS {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.imshow("AprilTag Vision (Python)", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        frame_id += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
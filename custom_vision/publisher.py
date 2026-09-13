"""NT4 results: one atomic JSON result plus convenient scalar/array topics."""
import json
import threading


class Publisher:
    def __init__(self, config, stdout=False):
        self.stdout = stdout
        self.lock = threading.Lock()
        self.instance = None
        self.tables = {}
        self.root = config.get("table", "/CustomVision")
        if config.get("enabled"):
            import ntcore
            self.instance = ntcore.NetworkTableInstance.create()
            self.instance.startClient4("CustomVision-Jetson")
            if config.get("server"):
                self.instance.setServer(config["server"])
            else:
                self.instance.setServerTeam(config["team"])

    def publish(self, payload):
        encoded = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        with self.lock:
            if self.stdout:
                print(encoded, flush=True)
            if self.instance is None:
                return
            name = payload["pipeline"]
            if name not in self.tables:
                table = self.instance.getTable(f"{self.root}/{name}")
                self.tables[name] = {
                    "result": table.getStringTopic("result").publish(),
                    "connected": table.getBooleanTopic("connected").publish(),
                    "has_target": table.getBooleanTopic("has_target").publish(),
                    "frame_id": table.getIntegerTopic("frame_id").publish(),
                    "count": table.getIntegerTopic("count").publish(),
                    "latency_ms": table.getDoubleTopic("latency_ms").publish(),
                    "tag_ids": table.getIntegerArrayTopic("tag_ids").publish(),
                }
            topics = self.tables[name]
            topics["result"].set(encoded)
            topics["connected"].set(payload["connected"])
            topics["has_target"].set(bool(payload["detections"]))
            topics["frame_id"].set(payload["frame_id"])
            topics["count"].set(len(payload["detections"]))
            topics["latency_ms"].set(payload["latency_ms"])
            topics["tag_ids"].set([d["id"] for d in payload["detections"] if "id" in d])
            self.instance.flush()

    def close(self):
        if self.instance:
            for topics in self.tables.values():
                for topic in topics.values():
                    topic.close()
            self.instance.stopClient()
            self.instance.destroy(self.instance)
            self.instance = None

import json

try:
    from networktables import NetworkTables
    NT_AVAILABLE = True
except Exception:
    NT_AVAILABLE = False

class Publisher:
    def __init__(self, publish_nt: bool, team_number: int, nt_table: str):
        self.publish_nt = publish_nt and NT_AVAILABLE
        self.table = None
        if self.publish_nt:
            server = f"10.{team_number // 100}.{team_number % 100}.2"
            NetworkTables.initialize(server=server)
            self.table = NetworkTables.getTable(nt_table)

    def publish(self, payload: dict):
        print(json.dumps(payload))
        if self.table is None:
            return
        self.table.putBoolean("connected", payload["connected"])
        self.table.putNumber("frame_id", payload["frame_id"])
        self.table.putNumber("timestamp_us", payload["timestamp_us"])
        self.table.putNumber("latency_ms", payload["latency_ms"])
        self.table.putNumber("fps", payload["fps"])
        self.table.putNumber("tag_count", payload["tag_count"])
        self.table.putString("json", json.dumps(payload))
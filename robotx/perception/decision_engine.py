class DecisionEngine:
    def __init__(self):
        self.state = "MOVE_FORWARD"

    def decide(self, primary):
        if primary is None:
            return "MOVE_FORWARD"

        # Fail-safe: if tracking is unstable, slow down.
        if primary and primary.missed > 2:
            return "SLOW"

        area = primary.area
        cx = primary.cx

        # Zones
        if cx < 213:
            pos = "LEFT"
        elif cx > 426:
            pos = "RIGHT"
        else:
            pos = "CENTER"

        # 🚨 Distance logic
        if area > 30000:
            if pos == "CENTER":
                return "STOP"
            elif pos == "LEFT":
                return "TURN_RIGHT"
            else:
                return "TURN_LEFT"

        if area > 10000:
            return "SLOW"

        return "MOVE_FORWARD"

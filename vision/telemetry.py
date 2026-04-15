from .config import TELEMETRY_SCENARIOS
from .models import TelemetrySample


class TelemetryProvider:
    def get_latest(self):
        raise NotImplementedError


class StaticTelemetryProvider(TelemetryProvider):
    def __init__(self, sample):
        self.sample = sample

    def get_latest(self):
        return self.sample

    @classmethod
    def from_scenario(cls, scenario_name):
        if scenario_name not in TELEMETRY_SCENARIOS:
            supported = ", ".join(sorted(TELEMETRY_SCENARIOS))
            raise ValueError(f"Unknown scenario {scenario_name!r}. Choose from: {supported}")
        return cls(TELEMETRY_SCENARIOS[scenario_name])

    @classmethod
    def from_values(cls, altitude_m, pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0):
        return cls(
            TelemetrySample(
                altitude_m=altitude_m,
                pitch_deg=pitch_deg,
                roll_deg=roll_deg,
                yaw_deg=yaw_deg,
            )
        )

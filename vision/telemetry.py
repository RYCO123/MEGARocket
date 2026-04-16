from .config import TELEMETRY_SCENARIOS
from .models import TelemetrySample


class TelemetryProvider:
    def get_latest(self) -> TelemetrySample:
        raise NotImplementedError


class LiveTelemetryProvider(TelemetryProvider):
    """
    Reads live altitude / pitch / roll from a callable at runtime.

    Pass the TelemetryReceiver.get method:
        LiveTelemetryProvider(telem_receiver.get)

    The callable must return a dict with keys: alt, pitch, roll, yaw.
    If altitude is 0 or missing (link not yet up) the fallback altitude
    is used so the projector keeps returning valid coordinates.
    """

    def __init__(self, get_fn, fallback_alt_m: float = 0.0):
        self._get_fn = get_fn
        self._fallback_alt_m = fallback_alt_m

    def get_latest(self) -> TelemetrySample:
        d = self._get_fn()
        alt = float(d.get('alt', 0.0))
        if alt <= 0.0:
            alt = self._fallback_alt_m
        return TelemetrySample(
            altitude_m=alt,
            pitch_deg=float(d.get('pitch', 0.0)),
            roll_deg=float(d.get('roll', 0.0)),
            yaw_deg=float(d.get('yaw', 0.0)),
        )


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

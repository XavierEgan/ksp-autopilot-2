import argparse
from dataclasses import dataclass
import time

import krpc
from krpc.client import Client
from krpc.services.spacecenter import Vessel

from dev_tools.grapher import ValueGrapher
from src.Telemetry import KRPCTelemetryProvider


@dataclass
class Settings:
    graph_telemetry: bool = False

def get_arguments() -> Settings:
    parser = argparse.ArgumentParser("dev tools for autopilot")
    _ = parser.add_argument("--graph_telemetry", "-gt", action="store_true")
    
    args = parser.parse_args()

    return Settings(
        **{k: v for k, v in args._get_kwargs() if k in Settings.__dict__}
    )

def graph_telemetry() -> None:
    conn: Client=  krpc.connect("graphing telemetry")
    vessel: Vessel | None = conn.space_center.active_vessel
    if vessel is None:
        print("You need to have an active vessel")
        return

    grapher: ValueGrapher = ValueGrapher(1000, cols=6)
    telemetry_provider: KRPCTelemetryProvider = KRPCTelemetryProvider(conn, vessel)
    i = 0

    while True:
        telemetry = telemetry_provider.get_telemetry()

        if telemetry.sample_age != 0.0:
            continue
        
        i += 1

        dont_graph: tuple[str, ...] = ("time", "dt", "sample_age", "on_ground", "terrain_height", "gear_agl", "dynamic_pressure", "density", "mach", "mass", "available_thrust")

        for v in telemetry.__dict__:
            if v in dont_graph:
                continue
            grapher.add_point(v, (telemetry.time, telemetry.__dict__[v]))
        
        if i % 10 == 0:
            grapher.update()
        

def main() -> None:
    settings: Settings = get_arguments()

    if settings.graph_telemetry:
        graph_telemetry()

if __name__ == "__main__":
    main()
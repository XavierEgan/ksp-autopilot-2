import argparse
from dataclasses import dataclass

@dataclass
class Settings:
    graph_derivative: bool = False

def get_arguments() -> Settings:
    parser = argparse.ArgumentParser("dev tools for autopilot")
    _ = parser.add_argument("--graph_derivative", "-gd", action="store_true")
    
    args = parser.parse_args()

    return Settings(
        **{k: v for k, v in args._get_kwargs() if k in Settings.__dict__}
    )

def graph_derivative() -> None:
    

def main() -> None:
    settings: Settings = get_arguments()

    if settings.graph_derivative:
        graph_derivative()

if __name__ == "__main__":
    main()
from pathlib import Path
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "python" / "config"


def load_yaml_config(filename: str) -> dict:
    config_path = CONFIG_DIR / filename

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    return data or {}
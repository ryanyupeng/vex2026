"""Build the one-file source downloaded to a VEX V5 Brain.

VEXcode V5 Python does not package sibling source modules. Keep the maintainable
fragments in src/ and run this script from the repository root before downloading.
"""
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SOURCE_DIR = ROOT / "src"
OUTPUT = ROOT / "hbase.v5python"
PARTS = (
    "01_robot_spec.py",
    "02_control_config.py",
    "04_helpers.py",
    "05_results.py",
    "06_pid.py",
    "07_settler.py",
    "08_hardware.py",
    "09_tracking.py",
    "10_drive.py",
    "11_mechanism.py",
    "11_navigation_path.py",
    "11_navigator.py",
    "12_robot.py",
    "13_route.py",
    "13_navigation_example.py",
    "14_main.py",
)


def build():
    missing = [name for name in PARTS if not (SOURCE_DIR / name).is_file()]
    if missing:
        raise FileNotFoundError("Missing source fragments: " + ", ".join(missing))

    header = '''"""Generated VEX deployment source. Do not edit directly.

Edit python/src/*.py and run python/build_vex.py.
"""
'''
    content = header + "\n".join((SOURCE_DIR / name).read_text() for name in PARTS)
    compile(content, str(OUTPUT), "exec")  # Reject invalid source before replacing output.
    OUTPUT.write_text(content)
    print("Built", OUTPUT, "from", len(PARTS), "source fragments")


if __name__ == "__main__":
    build()

"""Launch Lumos on terrain from OmniLRS's native Lunaryard generator."""

from pathlib import Path
import runpy


def generate_launch_description():
    shared = runpy.run_path(str(Path(__file__).with_name('site20.launch.py')))
    return shared['generate_launch_description'](map_name='lunaryard')

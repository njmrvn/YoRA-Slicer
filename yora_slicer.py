"""Start the YoRA Slicer from any folder:
    python path/to/YoRA/yora_slicer.py --open
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from yora.server import main  # noqa: E402

if __name__ == '__main__':
    main()

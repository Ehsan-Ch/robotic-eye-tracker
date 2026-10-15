"""Pixel-driven tracking for a simulated pan-tilt robotic eye."""
import os
_dll_handle=None
if os.name=='nt' and os.environ.get('EYE_LAB_DLL_DIR'):
    _dll_handle=os.add_dll_directory(os.environ['EYE_LAB_DLL_DIR'])

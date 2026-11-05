import logging, sys
logging.basicConfig(stream=sys.stderr, level="DEBUG")

# Then connect to the motors with the version that applies. Examples:
import pypot.dynamixel.io
pypot.dynamixel.io.DxlIO("COM3", baudrate=57600).scan()

import poppy_torso
poppy_torso.PoppyTorso()

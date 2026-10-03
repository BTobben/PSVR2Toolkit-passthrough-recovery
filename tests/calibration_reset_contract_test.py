"""Static integration guard; complements tests, not hardware acceptance."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / "projects/psvr2_openvr_driver_ex"


class CalibrationResetContract(unittest.TestCase):
    def test_calibration_cannot_reset_peer(self):
        source = (ROOT / "driver_hooks/libpad_hooks.cpp").read_text()
        self.assertNotIn("g_ShouldResetLEDTrackingInTicks", source)
        final = source.split("case CalibrationState::Finalize:", 1)[1]
        final = final.split("if (ctx.state != CalibrationState::Idle)", 1)[0]
        self.assertIn("libpad_SetSyncLedCommand(controllerTimeSync, controllerLedSync", final)
        self.assertIn("senseController.isLeft", final)
        self.assertIn("no shared optical reset", final)

    def test_frequency_reset_remains(self):
        source = (ROOT / "sense_controller.cpp").read_text()
        event = source.split("Prop_DisplayFrequency_Float", 1)[1].split("break;", 1)[0]
        self.assertIn("g_ShouldResetLEDTrackingInTicks = 150", event)


if __name__ == "__main__":
    unittest.main()

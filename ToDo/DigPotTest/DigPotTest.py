import unittest
from typing import Sequence

from smbus2 import SMBus

from DigPot import AD5142A, TPL0102, _discover_address, _env_int


def _next_test_value(current: int, delta: int) -> int:
	candidate = (current + delta) & 0xFF
	if candidate == current:
		candidate = (current + 1) & 0xFF
	return candidate


class DigipotHardwareTests(unittest.TestCase):
	@classmethod
	def setUpClass(cls) -> None:
		cls.bus_number = _env_int("DIGIPOT_I2C_BUS", 1)
		cls.bus = SMBus(cls.bus_number)
		cls.tpl0102 = TPL0102(
			cls.bus,
			_discover_address(cls.bus, "TPL0102_ADDRESS", TPL0102.CANDIDATE_ADDRESSES, "TPL0102"),
		)
		cls.ad5142a = AD5142A(
			cls.bus,
			_discover_address(cls.bus, "AD5142A_ADDRESS", AD5142A.CANDIDATE_ADDRESSES, "AD5142A"),
		)
		print(
			f"[setup] bus=/dev/i2c-{cls.bus_number} "
			f"TPL0102=0x{cls.tpl0102.address:02x} AD5142A=0x{cls.ad5142a.address:02x}",
			flush=True,
		)

	@classmethod
	def tearDownClass(cls) -> None:
		cls.bus.close()

	def _restore_tpl0102(self, values: Sequence[int]) -> None:
		for channel, value in enumerate(values):
			self.tpl0102.write_wiper(channel, value)

	def _restore_ad5142a(self, values: Sequence[int]) -> None:
		for channel, value in enumerate(values):
			self.ad5142a.write_rdac(channel, value)

	def _debug(self, message: str) -> None:
		print(f"[debug] {self.id()}: {message}", flush=True)

	@staticmethod
	def _format_values(values: Sequence[int]) -> str:
		return ", ".join(f"ch{channel}=0x{value:02x}" for channel, value in enumerate(values))

	def test_detects_expected_devices(self) -> None:
		self._debug(
			f"detected TPL0102 at 0x{self.tpl0102.address:02x}, "
			f"detected AD5142A at 0x{self.ad5142a.address:02x}"
		)
		self.assertIn(self.tpl0102.address, TPL0102.CANDIDATE_ADDRESSES)
		self.assertIn(self.ad5142a.address, AD5142A.CANDIDATE_ADDRESSES)

	def test_tpl0102_wipers_round_trip(self) -> None:
		original_acr = self.tpl0102.set_volatile_access(True)
		self.addCleanup(self.tpl0102.write_register, self.tpl0102.REG_ACR, original_acr)
		self._debug(
			f"TPL0102 ACR original=0x{original_acr:02x} "
			f"volatile-only=0x{self.tpl0102.read_register(self.tpl0102.REG_ACR):02x}"
		)

		original = [self.tpl0102.read_wiper(0), self.tpl0102.read_wiper(1)]
		self.addCleanup(self._restore_tpl0102, original)
		self._debug(f"TPL0102 original wipers: {self._format_values(original)}")

		expected = [
			_next_test_value(original[0], 0x11),
			_next_test_value(original[1], 0x37),
		]
		self._debug(f"TPL0102 target wipers:   {self._format_values(expected)}")

		self.tpl0102.write_wiper(0, expected[0])
		self.tpl0102.write_wiper(1, expected[1])

		actual = [self.tpl0102.read_wiper(0), self.tpl0102.read_wiper(1)]
		self._debug(f"TPL0102 readback wipers: {self._format_values(actual)}")

		self.assertEqual(actual[0], expected[0])
		self.assertEqual(actual[1], expected[1])

	def test_ad5142a_rdac_round_trip(self) -> None:
		original = [self.ad5142a.read_rdac(0), self.ad5142a.read_rdac(1)]
		self.addCleanup(self._restore_ad5142a, original)
		self._debug(f"AD5142A original RDAC: {self._format_values(original)}")

		expected = [
			_next_test_value(original[0], 0x21),
			_next_test_value(original[1], 0x43),
		]
		self._debug(f"AD5142A target RDAC:   {self._format_values(expected)}")

		self.ad5142a.write_rdac(0, expected[0])
		self.ad5142a.write_rdac(1, expected[1])

		actual = [self.ad5142a.read_rdac(0), self.ad5142a.read_rdac(1)]
		self._debug(f"AD5142A readback RDAC: {self._format_values(actual)}")

		self.assertEqual(actual[0], expected[0])
		self.assertEqual(actual[1], expected[1])


if __name__ == "__main__":
	unittest.main(verbosity=2)

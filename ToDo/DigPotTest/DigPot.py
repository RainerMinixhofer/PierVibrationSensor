import os
from typing import Iterable

from smbus2 import SMBus, i2c_msg


def _env_int(name: str, default: int | None = None) -> int | None:
	value = os.getenv(name)
	if value is None or value == "":
		return default
	return int(value, 0)


def _probe_address(bus: SMBus, address: int) -> bool:
	try:
		bus.write_quick(address)
		return True
	except OSError:
		pass

	try:
		bus.read_byte(address)
		return True
	except OSError:
		return False


def _discover_address(bus: SMBus, env_name: str, candidates: Iterable[int], label: str) -> int:
	configured = _env_int(env_name)
	if configured is not None:
		if not _probe_address(bus, configured):
			raise RuntimeError(
				f"{label} not reachable at {configured:#04x} from {env_name}."
			)
		return configured

	matches = [address for address in candidates if _probe_address(bus, address)]
	if not matches:
		candidate_text = ", ".join(f"0x{address:02x}" for address in candidates)
		raise RuntimeError(
			f"{label} not detected on I2C bus. Probed addresses: {candidate_text}."
		)
	if len(matches) > 1:
		match_text = ", ".join(f"0x{address:02x}" for address in matches)
		raise RuntimeError(
			f"Multiple {label} candidates found: {match_text}. Set {env_name} explicitly."
		)
	return matches[0]


class TPL0102:
	CANDIDATE_ADDRESSES = tuple(range(0x50, 0x58))
	REG_WRA = 0x00
	REG_WRB = 0x01
	REG_ACR = 0x10
	ACR_VOLATILE_ONLY = 0x80

	def __init__(self, bus: SMBus, address: int) -> None:
		self.bus = bus
		self.address = address

	def read_register(self, register: int) -> int:
		return self.bus.read_byte_data(self.address, register)

	def write_register(self, register: int, value: int) -> None:
		self.bus.write_byte_data(self.address, register, value & 0xFF)

	def set_volatile_access(self, enabled: bool) -> int:
		current = self.read_register(self.REG_ACR)
		updated = (current | self.ACR_VOLATILE_ONLY) if enabled else (current & ~self.ACR_VOLATILE_ONLY)
		if updated != current:
			self.write_register(self.REG_ACR, updated)
		return current

	def read_wiper(self, channel: int) -> int:
		register = self.REG_WRA if channel == 0 else self.REG_WRB
		return self.read_register(register)

	def write_wiper(self, channel: int, value: int) -> None:
		register = self.REG_WRA if channel == 0 else self.REG_WRB
		self.write_register(register, value)


class AD5142A:
	CANDIDATE_ADDRESSES = (0x20, 0x22, 0x23, 0x28, 0x2A, 0x2B, 0x2C, 0x2E, 0x2F)
	CHANNEL_ADDRESS = {0: 0x0, 1: 0x1}
	CMD_WRITE_RDAC = 0x1
	CMD_READBACK = 0x3
	READBACK_EEPROM = 0x1
	READBACK_RDAC = 0x3

	def __init__(self, bus: SMBus, address: int) -> None:
		self.bus = bus
		self.address = address

	@staticmethod
	def _build_word(command: int, address: int, data: int) -> int:
		return ((command & 0x0F) << 12) | ((address & 0x0F) << 8) | (data & 0xFF)

	def _write_word(self, word: int) -> None:
		write = i2c_msg.write(self.address, [(word >> 8) & 0xFF, word & 0xFF])
		self.bus.i2c_rdwr(write)

	def write_rdac(self, channel: int, value: int) -> None:
		address = self.CHANNEL_ADDRESS[channel]
		self._write_word(self._build_word(self.CMD_WRITE_RDAC, address, value))

	def read_rdac(self, channel: int) -> int:
		address = self.CHANNEL_ADDRESS[channel]
		self._write_word(self._build_word(self.CMD_READBACK, address, self.READBACK_RDAC))
		read = i2c_msg.read(self.address, 2)
		self.bus.i2c_rdwr(read)
		response = list(read) # type: ignore
		return response[0]

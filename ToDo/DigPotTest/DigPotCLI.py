#!/home/rainer/Dokumente/DigPotTest/.venv/bin/python3
"""Command-line interface for the TPL0102 and AD5142A digital potentiometers.

Supported Devices
-----------------

TPL0102 (Texas Instruments)
    • Dual-channel 256-tap I2C digital potentiometer
    • 100 kΩ end-to-end resistance (TPL0102-100 variant)
    • Non-volatile memory (EEPROM) for persistent wiper positions
    • Four control registers: WRA, WRB (volatile wipers), IVRA, IVRB (non-volatile)
    • Access Control Register (ACR) manages register bank switching and shutdown
    • Configurable I2C address via A2, A1, A0 pins → addresses 0x50–0x57

AD5142A (Analog Devices)
    • Dual-channel 256-tap I2C digital potentiometer
    • Supports 10 kΩ and 100 kΩ options
    • 16 advanced I2C commands (0–16) for flexible control
    • Volatile RDAC (wiper) and input (pre-load) registers
    • Non-volatile EEPROM and control register persistence
    • Software-selectable modes: potentiometer vs. linear gain, shutdown, reset
    • Configurable I2C address via ADDR0, ADDR1 pins → addresses 0x20–0x2F

Usage
-----
    DigPotCLI.py [global options] <device> <command> [command options]

Global Options
--------------
    --bus N          I2C bus number (default: 1, or $DIGIPOT_I2C_BUS)
    --tpl-addr 0xNN  Override TPL0102 I2C address (or $TPL0102_ADDRESS)
    --ad-addr 0xNN   Override AD5142A I2C address (or $AD5142A_ADDRESS)

Examples
--------
    # Read TPL0102 both channel wipers
    DigPotCLI.py tpl0102 wiper-read

    # Write both AD5142A RDAC channels to 0x80
    DigPotCLI.py ad5142a rdac-write 0x80

    # Set AD5142A channel 0 RDAC, save to EEPROM, reload it
    DigPotCLI.py ad5142a rdac-write -c 0 0x40
    DigPotCLI.py ad5142a rdac-to-eeprom -c 0
    DigPotCLI.py ad5142a eeprom-to-rdac -c 0

    # Poll TPL0102 until non-volatile write completes
    DigPotCLI.py tpl0102 wip-poll --timeout 1.0

Run ``DigPotCLI.py <device> --help`` for per-device help.
"""

import argparse
import sys
import time

from smbus2 import SMBus

from DigPot import (
    AD5142A,
    TPL0102,
    _discover_address,
    _env_int,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _open_bus(args: argparse.Namespace) -> SMBus:
    bus_number = getattr(args, "bus", None) or _env_int("DIGIPOT_I2C_BUS", 1)
    return SMBus(int(bus_number)) # type: ignore


def _open_tpl(bus: SMBus, args: argparse.Namespace) -> TPL0102:
    override = getattr(args, "tpl_addr", None)
    if override is not None:
        env_backup = "TPL0102_ADDRESS"
        import os
        os.environ[env_backup] = str(override)
    address = _discover_address(bus, "TPL0102_ADDRESS", TPL0102.CANDIDATE_ADDRESSES, "TPL0102")
    return TPL0102(bus, address)


def _open_ad(bus: SMBus, args: argparse.Namespace) -> AD5142A:
    override = getattr(args, "ad_addr", None)
    if override is not None:
        import os
        os.environ["AD5142A_ADDRESS"] = str(override)
    address = _discover_address(bus, "AD5142A_ADDRESS", AD5142A.CANDIDATE_ADDRESSES, "AD5142A")
    return AD5142A(bus, address)


def _channel(value: str) -> int:
    n = int(value, 0)
    if n not in (0, 1):
        raise argparse.ArgumentTypeError("channel must be 0 or 1")
    return n


def _byte_value(value: str) -> int:
    n = int(value, 0)
    if not (0 <= n <= 255):
        raise argparse.ArgumentTypeError("value must be 0–255 (0x00–0xFF)")
    return n


def _nibble(value: str) -> int:
    n = int(value, 0)
    if not (0 <= n <= 0xF):
        raise argparse.ArgumentTypeError("value must be 0–15 (0x0–0xF)")
    return n


def _print_result(label: str, value: int) -> None:
    print(f"{label}: 0x{value:02x}  ({value:3d} dec)")


AD5142A_RPOT_OHMS = 100000.0


def _ad_validate_voltdiv_inputs(vh: float, vl: float) -> None:
    if vh == vl:
        raise ValueError("VH and VL must be different")


def _ad_expected_voltage(
    position: int,
    vh_voltage: float,
    vl_voltage: float,
    rh_resistance: float,
    rl_resistance: float,
) -> float:
    return vl_voltage + (vh_voltage - vl_voltage) * (
        rl_resistance + AD5142A_RPOT_OHMS * (position / 255)
    ) / (rh_resistance + rl_resistance + AD5142A_RPOT_OHMS)


def _ad_position_for_voltage(
    voltage: float,
    vh_voltage: float,
    vl_voltage: float,
    rh_resistance: float,
    rl_resistance: float,
) -> int:
    numerator = (
        (voltage - vl_voltage)
        * (rh_resistance + rl_resistance + AD5142A_RPOT_OHMS)
        / (vh_voltage - vl_voltage)
        - rl_resistance
    )
    position = 255 * numerator / AD5142A_RPOT_OHMS
    return max(0, min(255, int(round(position))))


# ---------------------------------------------------------------------------
# TPL0102 sub-commands
# ---------------------------------------------------------------------------

def _tpl_wiper_read(dev: TPL0102, args: argparse.Namespace) -> None:
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        val = dev.read_wiper(ch)
        _print_result(f"TPL0102 Ch{ch} wiper (WR{'A' if ch == 0 else 'B'})", val)


def _tpl_wiper_write(dev: TPL0102, args: argparse.Namespace) -> None:
    dev.set_volatile_access(True)
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        dev.write_wiper(ch, args.value)
        print(f"TPL0102 Ch{ch} wiper set to 0x{args.value:02x}")


def _tpl_nvram_read(dev: TPL0102, args: argparse.Namespace) -> None:
    """Read initial-value (non-volatile) registers IVR A/B."""
    dev.set_volatile_access(False)
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        val = dev.read_register(TPL0102.REG_WRA if ch == 0 else TPL0102.REG_WRB)
        _print_result(f"TPL0102 Ch{ch} IVR{'A' if ch == 0 else 'B'} (non-volatile)", val)
    dev.set_volatile_access(True)


def _tpl_nvram_write(dev: TPL0102, args: argparse.Namespace) -> None:
    """Write initial-value (non-volatile) registers IVR A/B."""
    dev.set_volatile_access(False)
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        dev.write_register(TPL0102.REG_WRA if ch == 0 else TPL0102.REG_WRB, args.value)
        print(f"TPL0102 Ch{ch} IVR{'A' if ch == 0 else 'B'} set to 0x{args.value:02x}")
    dev.set_volatile_access(True)


def _tpl_acr_read(dev: TPL0102, _args: argparse.Namespace) -> None:
    raw = dev.read_register(TPL0102.REG_ACR)
    vol  = (raw >> 7) & 1
    shdn = (raw >> 6) & 1
    wip  = (raw >> 1) & 1
    _print_result("TPL0102 ACR raw", raw)
    print(f"  VOL  (bit7) = {vol}  ({'volatile-only' if vol else 'non-volatile accessible'})")
    print(f"  SHDN (bit6) = {shdn}  ({'shutdown disabled' if shdn else 'SHUTDOWN ACTIVE'})")
    print(f"  WIP  (bit1) = {wip}  ({'NV write in progress' if wip else 'ready'})")


def _tpl_acr_write(dev: TPL0102, args: argparse.Namespace) -> None:
    dev.write_register(TPL0102.REG_ACR, args.value)
    print(f"TPL0102 ACR set to 0x{args.value:02x}")


def _tpl_shutdown(dev: TPL0102, args: argparse.Namespace) -> None:
    acr = dev.read_register(TPL0102.REG_ACR)
    if args.enable:
        acr &= ~0x40        # clear SHDN bit → shutdown active
        print("TPL0102 entering shutdown mode")
    else:
        acr |= 0x40         # set SHDN bit → normal operation
        print("TPL0102 leaving shutdown mode")
    dev.write_register(TPL0102.REG_ACR, acr)


def _tpl_wip_poll(dev: TPL0102, args: argparse.Namespace) -> None:
    """Poll WIP bit until NV write is complete."""
    timeout = args.timeout
    interval = 0.010
    elapsed = 0.0
    while True:
        acr = dev.read_register(TPL0102.REG_ACR)
        wip = (acr >> 1) & 1
        if not wip:
            print(f"TPL0102 NV write complete (elapsed {elapsed:.3f}s)")
            return
        if elapsed >= timeout:
            print(f"TPL0102 WIP still set after {elapsed:.3f}s timeout", file=sys.stderr)
            sys.exit(1)
        time.sleep(interval)
        elapsed += interval


# ---------------------------------------------------------------------------
# AD5142A sub-commands
# ---------------------------------------------------------------------------

def _ad_rdac_write(dev: AD5142A, args: argparse.Namespace) -> None:
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        dev.write_rdac(ch, args.value)
        print(f"AD5142A Ch{ch} RDAC set to 0x{args.value:02x}")


def _ad_rdac_read(dev: AD5142A, args: argparse.Namespace) -> None:
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        val = dev.read_rdac(ch)
        _print_result(f"AD5142A Ch{ch} RDAC", val)


def _ad_input_write(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 2: write input (pre-load) register."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        dev._write_word(dev._build_word(0x2, address, args.value))
        print(f"AD5142A Ch{ch} input register set to 0x{args.value:02x}")


def _ad_input_to_rdac(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 8: copy input register to RDAC (software LRDAC)."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        dev._write_word(dev._build_word(0x8, address, 0x00))
        print(f"AD5142A Ch{ch} input register copied to RDAC")


def _ad_readback(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 3: read back input register, EEPROM, control register, or RDAC."""
    source_map = {"input": 0x0, "eeprom": 0x1, "control": 0x2, "rdac": 0x3}
    d1d0 = source_map[args.source]
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        dev._write_word(dev._build_word(0x3, address, d1d0))
        from smbus2 import i2c_msg
        read = i2c_msg.read(dev.address, 2)
        dev.bus.i2c_rdwr(read)
        val = list(read)[0] # type: ignore
        _print_result(f"AD5142A Ch{ch} {args.source}", val)


def _ad_eeprom_write(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 11: write EEPROM directly."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        a1a0 = ch & 0x3
        dev._write_word(dev._build_word(0xB, a1a0, args.value))
        print(f"AD5142A Ch{ch} EEPROM set to 0x{args.value:02x}")


def _ad_rdac_to_eeprom(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 9: copy RDAC register to EEPROM."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        dev._write_word(dev._build_word(0x9, address, 0x01))
        print(f"AD5142A Ch{ch} RDAC copied to EEPROM")


def _ad_eeprom_to_rdac(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 10: copy EEPROM into RDAC."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        dev._write_word(dev._build_word(0xA, address, 0x00))
        print(f"AD5142A Ch{ch} EEPROM copied to RDAC")


def _ad_increment(dev: AD5142A, args: argparse.Namespace) -> None:
    """Commands 4/5: linear increment or decrement."""
    step_bit = 0 if args.direction == "dec" else 1
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        for _ in range(args.steps):
            dev._write_word(dev._build_word(0x4, address, step_bit))
        print(f"AD5142A Ch{ch} linear {'inc' if step_bit else 'dec'}rement x{args.steps}")


def _ad_6db(dev: AD5142A, args: argparse.Namespace) -> None:
    """Commands 6/7: ±6 dB increment or decrement."""
    step_bit = 0 if args.direction == "dec" else 1
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        for _ in range(args.steps):
            dev._write_word(dev._build_word(0x5, address, step_bit))
        print(f"AD5142A Ch{ch} 6dB {'inc' if step_bit else 'dec'}rement x{args.steps}")


def _ad_top_scale(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 12: set top scale / clear shutdown."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        d0 = 1 if args.shutdown else 0
        dev._write_word(dev._build_word(0xC, address, 0x80 | d0))
        print(f"AD5142A Ch{ch} top scale (shutdown={'on' if d0 else 'off'})")


def _ad_bottom_scale(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 13: set bottom scale."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        d0 = 1 if args.enter else 0
        dev._write_word(dev._build_word(0xD, address, d0))
        print(f"AD5142A Ch{ch} bottom scale ({'enter' if d0 else 'exit'})")


def _ad_reset(dev: AD5142A, _args: argparse.Namespace) -> None:
    """Command 14: software reset (reload EEPROM into all RDACs)."""
    dev._write_word(dev._build_word(0xE, 0x0, 0x00))
    print("AD5142A software reset issued")


def _ad_shutdown(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 15: software shutdown."""
    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        address = dev.CHANNEL_ADDRESS[ch]
        d0 = 1 if args.enable else 0
        dev._write_word(dev._build_word(0xF, address, d0))
        print(f"AD5142A Ch{ch} shutdown {'enabled' if d0 else 'disabled'}")


def _ad_control_write(dev: AD5142A, args: argparse.Namespace) -> None:
    """Command 16: write control register (D3:D0 = burst, lgs, eeprom-en, wp)."""
    d3d0 = args.value & 0x0F
    dev._write_word(dev._build_word(0x10, 0x0, d3d0))
    wp    = (d3d0 >> 0) & 1
    ee_en = (d3d0 >> 1) & 1
    lgs   = (d3d0 >> 2) & 1
    burst = (d3d0 >> 3) & 1
    print(
        f"AD5142A control register set to 0x{d3d0:01x}  "
        f"(WP={wp} EE_EN={ee_en} LGS={lgs} BURST={burst})"
    )


def _ad_nop(dev: AD5142A, _args: argparse.Namespace) -> None:
    """Command 0: NOP."""
    dev._write_word(dev._build_word(0x0, 0x0, 0x00))
    print("AD5142A NOP sent")


def _ad_voltdiv_write(dev: AD5142A, args: argparse.Namespace) -> None:
    """Shortcut: calculate and set wiper position for voltage divider configuration."""
    vh = args.vh_voltage
    vl = args.vl_voltage
    rh = args.rh_resistance
    rl = args.rl_resistance
    vout = args.voltage

    try:
        _ad_validate_voltdiv_inputs(vh, vl)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return

    position = _ad_position_for_voltage(vout, vh, vl, rh, rl)
    vout_actual = _ad_expected_voltage(position, vh, vl, rh, rl)

    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        dev.write_rdac(ch, position)
        error = abs(vout_actual - vout)
        print(
            f"AD5142A Ch{ch} voltdiv-write: target={vout:.4f}V, "
            f"position={position} (0x{position:02x}), actual={vout_actual:.4f}V, "
            f"error={error:.4f}V"
        )


def _ad_voltdiv_read(dev: AD5142A, args: argparse.Namespace) -> None:
    """Shortcut: calculate expected voltage from the current wiper position."""
    vh = args.vh_voltage
    vl = args.vl_voltage
    rh = args.rh_resistance
    rl = args.rl_resistance

    try:
        _ad_validate_voltdiv_inputs(vh, vl)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return

    channels = [args.channel] if args.channel is not None else [0, 1]
    for ch in channels:
        position = dev.read_rdac(ch)
        voltage = _ad_expected_voltage(position, vh, vl, rh, rl)
        print(
            f"AD5142A Ch{ch} voltdiv-read: position={position} (0x{position:02x}), "
            f"expected={voltage:.4f}V"
        )


# ---------------------------------------------------------------------------
# Argument parser construction
# ---------------------------------------------------------------------------

def _build_tpl_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser(
        "tpl0102",
        help="Control TPL0102 dual-channel 256-tap I2C potentiometer",
        description="""
TPL0102 Operations
------------------
The TPL0102 has two independent channels (Ch0, Ch1), each with:
  • WRA/WRB: Volatile wiper register (changes on power-off)
  • IVRA/IVRB: Non-volatile initial-value register (survives power-off)
  • ACR: Access Control Register (VOL, SHDN, WIP bits)

Register Banks:
  • VOL=0: R/W accesses IVRA/IVRB (non-volatile); writes also update WRA/WRB
  • VOL=1: R/W accesses only WRA/WRB (volatile); reads from IVRA/IVRB only

Typical Workflow:
  1. wiper-read          → Get current volatile wiper
  2. wiper-write VALUE   → Set wiper to new position
  3. nvram-write VALUE   → Save position to EEPROM
  4. (power-off)
  5. (power-on) → device auto-restores from EEPROM to both WRA/WRB and IVRA/IVRB
""",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    cmds = p.add_subparsers(dest="tpl_cmd", required=True, metavar="COMMAND")

    # wiper-read
    c = cmds.add_parser(
        "wiper-read",
        help="Read volatile wiper register (WRA / WRB)",
        description="Read the current wiper position from the volatile register. "
                    "Accessible only when ACR VOL bit is set to 1."
    )
    c.add_argument("--channel", "-c", type=_channel, default=None,
                   metavar="{0,1}", help="Channel (default: both)")

    # wiper-write
    c = cmds.add_parser("wiper-write", help="Write volatile wiper register (WRA / WRB)")
    c.add_argument("--channel", "-c", type=_channel, default=None,
                   metavar="{0,1}", help="Channel (default: both)")
    c.add_argument("value", type=_byte_value, metavar="VALUE",
                   help="Wiper position 0–255")

    # nvram-read
    c = cmds.add_parser("nvram-read",
                        help="Read non-volatile initial-value register (IVRA / IVRB)")
    c.add_argument("--channel", "-c", type=_channel, default=None,
                   metavar="{0,1}", help="Channel (default: both)")

    # nvram-write
    c = cmds.add_parser("nvram-write",
                        help="Write non-volatile initial-value register (IVRA / IVRB)")
    c.add_argument("--channel", "-c", type=_channel, default=None,
                   metavar="{0,1}", help="Channel (default: both)")
    c.add_argument("value", type=_byte_value, metavar="VALUE",
                   help="Wiper position 0–255")

    # acr-read
    cmds.add_parser(
        "acr-read",
        help="Read and decode Access Control Register",
        description="Display ACR contents and all bit meanings: "
                    "VOL (bit 7), SHDN (bit 6), WIP (bit 1)"
    )

    # acr-write
    c = cmds.add_parser("acr-write", help="Write raw byte to Access Control Register")
    c.add_argument("value", type=_byte_value, metavar="VALUE",
                   help="Raw ACR byte (see datasheet)")

    # shutdown
    c = cmds.add_parser("shutdown", help="Enter or exit shutdown mode (ACR SHDN bit)")
    g = c.add_mutually_exclusive_group(required=True)
    g.add_argument("--enable",  dest="enable", action="store_true",  help="Enter shutdown")
    g.add_argument("--disable", dest="enable", action="store_false", help="Exit shutdown")

    # wip-poll
    c = cmds.add_parser(
        "wip-poll",
        help="Poll WIP bit until EEPROM write completes",
        description="After nvram-write, the TPL0102 sets WIP (bit 1) to 1 while EEPROM "
                    "is being programmed (~10 ms). This command polls ACR WIP until "
                    "clear (indicating write complete) or timeout expires. "
                    "Use --timeout N (default 0.5 seconds) to set max wait time."
    )
    c.add_argument("--timeout", type=float, default=0.5, metavar="SECONDS",
                   help="Max wait time in seconds (default: 0.5)")


def _build_ad_parser(sub: argparse._SubParsersAction) -> None:
    p = sub.add_parser(
        "ad5142a",
        help="Control AD5142A dual-channel 256-tap I2C potentiometer",
        description="""
AD5142A Operations
------------------
The AD5142A has two independent channels (Ch0, Ch1), each with:
  • RDAC: Volatile wiper register (changes on power-off)
  • Input Register: Pre-load buffer for synchronous updates
  • EEPROM: Non-volatile storage (survives power-off)
  • Control Register: Software modes (WP, EE_EN, LGS, BURST)

Command Set (16 commands, 0–16):
  0  NOP                          Do nothing
  1  rdac-write                   Write RDAC directly (volatile)
    -  voltdiv-write                Set RDAC from target divider voltage
    -  voltdiv-read                 Read expected divider voltage from RDAC
  2  input-write                  Pre-load input register for sync update
  3  readback                     Read RDAC, EEPROM, input, or control (source=...)
  4/5 linear-step                 Step RDAC up/down linearly
  6/7 6db-step                    Step RDAC up/down in 6dB increments
  8  input-to-rdac                Copy input register to RDAC (software LRDAC)
  9  rdac-to-eeprom               Save RDAC value to EEPROM
  10 eeprom-to-rdac               Reload RDAC from EEPROM
  11 eeprom-write                 Write EEPROM directly (slow)
  12 top-scale                    Set RDAC to top/maximum value
  13 bottom-scale                 Set RDAC to bottom/minimum value
  14 reset                        Software reset (reload all from EEPROM)
  15 shutdown                     Per-channel software shutdown
  16 control-write                Set control register flags

Typical Workflow:
  1. rdac-write VALUE             → Set wiper to new position
  2. rdac-to-eeprom               → Save to persistent memory
  3. (power-off)
  4. (power-on) → auto-reload from EEPROM via reset command if needed
""",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    cmds = p.add_subparsers(dest="ad_cmd", required=True, metavar="COMMAND")

    def _ch_opt(c: argparse.ArgumentParser) -> None:
        c.add_argument("--channel", "-c", type=_channel, default=None,
                       metavar="{0,1}", help="Channel (default: both)")

    # nop
    cmds.add_parser(
        "nop",
        help="Command 0: NOP (no operation)",
        description="Send no-operation command. Useful for timing checks or to verify "
                    "I2C bus communication without affecting any register state."
    )

    # rdac-read  (shortcut for readback rdac)
    c = cmds.add_parser(
        "rdac-read",
        help="Read current RDAC wiper position (Command 3 shortcut)",
        description="Shortcut for 'readback rdac'; reads the volatile RDAC register "
                    "containing the current wiper position (0–255). Does not read EEPROM."
    )
    _ch_opt(c)

    # rdac-write
    c = cmds.add_parser(
        "rdac-write",
        help="Command 1: write RDAC wiper position",
        description="Set wiper position in volatile RDAC register (0–255). "
                    "Does not affect EEPROM; use rdac-to-eeprom to persist. "
                    "Use -c/--channel to limit to one channel, or omit to affect both."
    )
    _ch_opt(c)
    c.add_argument("value", type=_byte_value, metavar="VALUE", help="Wiper position 0–255")

    # input-write
    c = cmds.add_parser(
        "input-write",
        help="Command 2: write input (pre-load) register",
        description="Load a value into the input register (0–255). This serves as a "
                    "pre-load accumulator for use with input-to-rdac (software LRDAC). "
                    "Does not directly affect wiper position until input-to-rdac is issued."
    )
    _ch_opt(c)
    c.add_argument("value", type=_byte_value, metavar="VALUE", help="Value 0–255")

    # readback
    c = cmds.add_parser(
        "readback",
        help="Command 3: read back input, EEPROM, control register, or RDAC",
        description="Read selected register back to host. Available sources: "
                    "input (pre-load register), eeprom (non-volatile), "
                    "control (GEN, LGS, BURST, EE_EN bits), "
                    "rdac (volatile wiper position). Specify source as positional argument."
    )
    _ch_opt(c)
    c.add_argument("source",
                   choices=["input", "eeprom", "control", "rdac"],
                   help="Data source to read back")

    # increment / decrement
    c = cmds.add_parser(
        "linear-step",
        help="Commands 4/5: RDAC linear increment or decrement",
        description="Adjust RDAC value one step at a time (each step = 1 LSB, ~390Ω for 100kΩ "
                    "potentiometer). Use 'inc' or 'dec' to specify direction. "
                    "Use --steps N to apply multiple steps in one command. Wraps at boundaries."
    )
    _ch_opt(c)
    c.add_argument("direction", choices=["inc", "dec"], help="Direction")
    c.add_argument("--steps", "-n", type=int, default=1, metavar="N",
                   help="Number of steps (default: 1)")

    # ±6 dB
    c = cmds.add_parser(
        "6db-step",
        help="Commands 6/7: RDAC ±6 dB increment or decrement",
        description="Apply 6 dB logarithmic step (double/halve attenuation). "
                    "Useful for audio volume control with perceptually uniform steps. "
                    "Use --steps N to apply multiple 6 dB steps in one command."
    )
    _ch_opt(c)
    c.add_argument("direction", choices=["inc", "dec"], help="Direction")
    c.add_argument("--steps", "-n", type=int, default=1, metavar="N",
                   help="Number of steps (default: 1)")

    # input-to-rdac
    c = cmds.add_parser(
        "input-to-rdac",
        help="Command 8: copy input register to RDAC (software LRDAC)",
        description="Transfer the value from the input (pre-load) register to RDAC, "
                    "updating wiper position in one operation. Emulates a latched "
                    "input (LRDAC) behavior in software."
    )
    _ch_opt(c)

    # rdac-to-eeprom
    c = cmds.add_parser(
        "rdac-to-eeprom",
        help="Command 9: copy RDAC register to EEPROM",
        description="Save the current wiper position (RDAC) to non-volatile EEPROM. "
                    "EEPROM write takes ~10 ms. This value will be restored to RDAC "
                    "on power-up or after reset command."
    )
    _ch_opt(c)

    # eeprom-to-rdac
    c = cmds.add_parser(
        "eeprom-to-rdac",
        help="Command 10: copy EEPROM into RDAC",
        description="Restore the saved EEPROM value back into the volatile RDAC register, "
                    "updating wiper position. Equivalent to power-on initialization. "
                    "Useful to reload a previously saved wiper position."
    )
    _ch_opt(c)

    # eeprom-write
    c = cmds.add_parser(
        "eeprom-write",
        help="Command 11: write EEPROM directly",
        description="Write a value directly into EEPROM without modifying RDAC. "
                    "EEPROM write takes ~10 ms. Next power-up or reset will load "
                    "this value into RDAC. Useful for pre-loading a specific state."
    )
    _ch_opt(c)
    c.add_argument("value", type=_byte_value, metavar="VALUE", help="Value 0–255")

    # top-scale
    c = cmds.add_parser(
        "top-scale",
        help="Command 12: set wiper to top scale (max resistance)",
        description="Set RDAC to 0xFF (255) immediately, placing wiper at maximum. "
                    "Optional: use --shutdown to simultaneously enter shutdown mode "
                    "(zeroes load on output). Useful for initialization or safe idle state."
    )
    _ch_opt(c)
    c.add_argument("--shutdown", action="store_true",
                   help="Also set shutdown mode (D0=1)")

    # bottom-scale
    c = cmds.add_parser(
        "bottom-scale",
        help="Command 13: set wiper to bottom scale (min resistance)",
        description="Set RDAC to 0x00 immediately, placing wiper at minimum. "
                    "Use --enter to enter bottom-scale mode (persistent; wiper locked at 0); "
                    "without flag, command exits bottom-scale mode. Used for safe minimum "
                    "position or temporary operation constraint."
    )
    _ch_opt(c)
    c.add_argument("--enter", action="store_true",
                   help="Enter bottom scale (D0=1); without flag: exit")

    # reset
    cmds.add_parser(
        "reset",
        help="Command 14: software reset",
        description="Perform software reset: reload EEPROM values into all RDAC registers "
                    "for both channels. Clears all volatile state (shutdown, scales, etc). "
                    "Equivalent to power-on initialization."
    )

    # shutdown
    c = cmds.add_parser(
        "shutdown",
        help="Command 15: software shutdown per channel",
        description="Enable or disable shutdown mode for selected channel. "
                    "Shutdown zeroes the load on the potentiometer output and reduces "
                    "power consumption. Use --enable to enter, --disable to exit. "
                    "Omit -c/--channel to affect both channels."
    )
    _ch_opt(c)
    g = c.add_mutually_exclusive_group(required=True)
    g.add_argument("--enable",  dest="enable", action="store_true",  help="Enter shutdown")
    g.add_argument("--disable", dest="enable", action="store_false", help="Exit shutdown")

    # control-write
    c = cmds.add_parser(
        "control-write",
        help="Command 16: write control register (GEN, LGS, EE_EN, WP bits)",
        description="Set individual control bits (4-bit value 0x0–0xF): "
                    "D3=BURST (extended address mode), D2=LGS (logic ground select), "
                    "D1=EE_EN (EEPROM enable for direct writes), D0=WP (write protect). "
                    "Consult datasheet for typical settings per application."
    )
    c.add_argument("value", type=_nibble, metavar="VALUE",
                   help="4-bit value 0–15 (0x0–0xF)")

    # voltdiv-write (shortcut)
    c = cmds.add_parser(
        "voltdiv-write",
        help="Shortcut: set wiper for voltage divider configuration",
        description="Calculate and set wiper position to achieve a target voltage in "
                    "a resistive voltage divider with fixed series resistors on the "
                    "supply rails. Assumes AD5142A 100kΩ potentiometer. "
                    "Formula: Vout = VL + (VH - VL) * (RL + Rpot*(1-P/255)) / (RH + RL + Rpot). "
                    "Specify voltages in volts (e.g. 5.0, -3.3) and resistances in ohms (e.g. 1000). "
                    "Use -c/--channel to select channel; omit to set both."
    )
    _ch_opt(c)
    c.add_argument("--vh-voltage", type=float, default=5.0, metavar="V",
                   help="High-side supply voltage (volts, default: 5.0)")
    c.add_argument("--vl-voltage", type=float, default=0.0, metavar="V",
                   help="Low-side supply voltage (volts, default: 0.0)")
    c.add_argument("--rh-resistance", type=float, default=1000.0, metavar="OHM",
                   help="High-side fixed series resistor (ohms, default: 1000.0)")
    c.add_argument("--rl-resistance", type=float, default=0.0, metavar="OHM",
                   help="Low-side fixed series resistor (ohms, default: 0.0)")
    c.add_argument("voltage", type=float, nargs="?", default=2.5, metavar="V",
                   help="Target voltage at wiper (volts, default: 2.5)")

    # voltdiv-read (shortcut)
    c = cmds.add_parser(
        "voltdiv-read",
        help="Shortcut: read expected voltage for voltage divider configuration",
        description="Read the current RDAC wiper position and calculate the expected "
                    "wiper voltage for the same resistive divider model used by "
                    "voltdiv-write. Assumes AD5142A 100kΩ potentiometer. "
                    "Formula: Vout = VL + (VH - VL) * (RL + Rpot*(1-P/255)) / (RH + RL + Rpot). "
                    "Specify voltages in volts (e.g. 5.0, -3.3) and resistances in ohms (e.g. 1000). "
                    "Use -c/--channel to select channel; omit to read both."
    )
    _ch_opt(c)
    c.add_argument("--vh-voltage", type=float, default=5.0, metavar="V",
                   help="High-side supply voltage (volts, default: 5.0)")
    c.add_argument("--vl-voltage", type=float, default=0.0, metavar="V",
                   help="Low-side supply voltage (volts, default: 0.0)")
    c.add_argument("--rh-resistance", type=float, default=1000.0, metavar="OHM",
                   help="High-side fixed series resistor (ohms, default: 1000.0)")
    c.add_argument("--rl-resistance", type=float, default=0.0, metavar="OHM",
                   help="Low-side fixed series resistor (ohms, default: 0.0)")


# ---------------------------------------------------------------------------
# Main dispatcher
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="DigPotCLI",
        description="CLI for TPL0102 and AD5142A digital potentiometers via I2C",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--bus", type=int, default=None, metavar="N",
                        help="I2C bus number (default: 1 or $DIGIPOT_I2C_BUS)")
    parser.add_argument("--tpl-addr", type=lambda x: int(x, 0), default=None,
                        metavar="ADDR", help="Override TPL0102 I2C address")
    parser.add_argument("--ad-addr", type=lambda x: int(x, 0), default=None,
                        metavar="ADDR", help="Override AD5142A I2C address")

    sub = parser.add_subparsers(dest="device", required=True, metavar="DEVICE")
    _build_tpl_parser(sub)
    _build_ad_parser(sub)

    args = parser.parse_args(argv)
    bus = _open_bus(args)

    try:
        if args.device == "tpl0102":
            dev = _open_tpl(bus, args)
            dispatch = {
                "wiper-read":   _tpl_wiper_read,
                "wiper-write":  _tpl_wiper_write,
                "nvram-read":   _tpl_nvram_read,
                "nvram-write":  _tpl_nvram_write,
                "acr-read":     _tpl_acr_read,
                "acr-write":    _tpl_acr_write,
                "shutdown":     _tpl_shutdown,
                "wip-poll":     _tpl_wip_poll,
            }
            dispatch[args.tpl_cmd](dev, args)

        elif args.device == "ad5142a":
            dev = _open_ad(bus, args)
            dispatch = {
                "nop":            _ad_nop,
                "rdac-write":     _ad_rdac_write,
                "input-write":    _ad_input_write,
                "readback":       _ad_readback,
                "linear-step":    _ad_increment,
                "6db-step":       _ad_6db,
                "input-to-rdac":  _ad_input_to_rdac,
                "rdac-to-eeprom": _ad_rdac_to_eeprom,
                "eeprom-to-rdac": _ad_eeprom_to_rdac,
                "eeprom-write":   _ad_eeprom_write,
                "top-scale":      _ad_top_scale,
                "bottom-scale":   _ad_bottom_scale,
                "reset":          _ad_reset,
                "shutdown":       _ad_shutdown,
                "control-write":  _ad_control_write,
                "rdac-read":      _ad_rdac_read,
                "voltdiv-write":  _ad_voltdiv_write,
                "voltdiv-read":   _ad_voltdiv_read,
            }
            dispatch[args.ad_cmd](dev, args)

    finally:
        bus.close()


if __name__ == "__main__":
    main()

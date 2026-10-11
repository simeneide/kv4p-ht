# PlatformIO extra script: name this build in the ESP-IDF app descriptor.
#
# Arduino-ESP32 links a precompiled esp_app_desc_t, so every image reports
# version "esp-idf: v4.4.7 ..." and project "arduino-lib-builder", and a
# flasher or `esptool.py image_info` cannot tell one kv4p build from another.
# This rewrites the descriptor's version and project_name fields in the linked
# ELF, before elf2image turns it into firmware.bin, so the image checksum, the
# appended SHA-256 and the ELF SHA in the descriptor all cover the new bytes.
# The version string is FIRMWARE_LABEL from firmwareVersion.h, the same label
# the firmware prints at boot. Nothing the firmware sends over the air changes.

import re
import struct
from pathlib import Path

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

PROJECT_NAME = b"kv4p-ht-pgpilot"
APP_DESC_MAGIC = 0xABCD5432
VERSION_OFFSET = 16
PROJECT_NAME_OFFSET = 48
FIELD_LEN = 32


def firmware_label() -> bytes:
    header = Path(env.subst("$PROJECT_SRC_DIR")) / "firmwareVersion.h"  # noqa: F821
    match = re.search(r'#define\s+FIRMWARE_LABEL\s+"([^"]+)"', header.read_text())
    if match is None:
        raise RuntimeError(f"FIRMWARE_LABEL not found in {header}")
    return match.group(1).encode()


def app_desc_file_offset(elf: bytes) -> int:
    # ELF32 little-endian: find the .flash.appdesc section by name.
    shoff = struct.unpack_from("<I", elf, 0x20)[0]
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", elf, 0x2E)
    strtab_offset = struct.unpack_from("<I", elf, shoff + shstrndx * shentsize + 0x10)[0]
    for i in range(shnum):
        name_index, _, _, _, offset = struct.unpack_from("<IIIII", elf, shoff + i * shentsize)
        end = elf.index(b"\0", strtab_offset + name_index)
        if elf[strtab_offset + name_index : end] == b".flash.appdesc":
            return offset
    raise RuntimeError("no .flash.appdesc section in the ELF")


def set_app_version(target, source, env):  # noqa: ARG001
    elf_path = Path(str(target[0]))
    elf = bytearray(elf_path.read_bytes())
    desc = app_desc_file_offset(bytes(elf))
    if struct.unpack_from("<I", elf, desc)[0] != APP_DESC_MAGIC:
        raise RuntimeError("app descriptor magic not found at .flash.appdesc")
    label = firmware_label()
    for offset, value in ((VERSION_OFFSET, label), (PROJECT_NAME_OFFSET, PROJECT_NAME)):
        if len(value) >= FIELD_LEN:
            raise RuntimeError(f"{value!r} does not fit in the app descriptor")
        elf[desc + offset : desc + offset + FIELD_LEN] = value.ljust(FIELD_LEN, b"\0")
    elf_path.write_bytes(bytes(elf))
    print(f"App descriptor: version={label.decode()} project={PROJECT_NAME.decode()}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", set_app_version)  # noqa: F821

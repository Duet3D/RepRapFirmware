#!/usr/bin/env python3
"""Append the firmware CRC that RepRapFirmware checks at startup.

A drop-in replacement for the Duet3D CrcAppender binary that the board Makefiles call if it is on the
PATH. Without it the .bin is unflashable: AppMain (Platform/Tasks.cpp) recomputes this CRC on every
boot and, if it does not match, sits in a 3-blink diagnostic loop instead of starting.

Vector table slot 7 (offset 0x1C) holds &_firmware_crc, the absolute address the CRC word lives at.
The image starts at the vector table, so the CRC offset within the file is that address minus the
load address, which for a correctly linked image is exactly the current end of the file. The CRC is
the standard reflected CRC-32 (poly 0xEDB88320, init 0xFFFFFFFF, final complement) that
src/Storage/CRC32.cpp implements, i.e. the one in zlib.

Usage: CrcAppender.py <firmware.bin> [--load-address 0x400000]
"""

import argparse
import struct
import sys
import zlib

VECTOR_SLOT_FIRMWARE_CRC = 0x1C


def append_crc(path, load_address):
    with open(path, 'rb') as f:
        image = f.read()

    if len(image) < VECTOR_SLOT_FIRMWARE_CRC + 4:
        raise SystemExit(f"{path}: too short to contain a vector table")

    crc_address, = struct.unpack_from('<I', image, VECTOR_SLOT_FIRMWARE_CRC)
    if crc_address == 0:
        raise SystemExit(f"{path}: vector slot 7 is zero - this image was not linked with a firmware CRC")

    crc_offset = crc_address - load_address
    if crc_offset != len(image):
        # Anything else means we would be CRCing the wrong extent, so refuse rather than write a
        # wrong-but-plausible value that only shows up as a blink code on the bench.
        raise SystemExit(
            f"{path}: CRC address {crc_address:#x} implies offset {crc_offset:#x} "
            f"but the image is {len(image):#x} bytes. Wrong --load-address, or a CRC has already been appended.")

    crc = zlib.crc32(image) & 0xFFFFFFFF
    with open(path, 'ab') as f:
        f.write(struct.pack('<I', crc))
    return crc, crc_address


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('binary')
    parser.add_argument('--load-address', default='0x400000',
                        help='address the image is linked at (default 0x400000, the SAME70/SAM4E flash base)')
    args = parser.parse_args()

    crc, crc_address = append_crc(args.binary, int(args.load_address, 0))
    print(f"{args.binary}: CRC {crc:#010x} appended at {crc_address:#x}", file=sys.stderr)


if __name__ == '__main__':
    main()

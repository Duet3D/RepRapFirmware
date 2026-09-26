#!/usr/bin/env python3
"""Append an embedded filesystem to a USE_EMBEDDED_FILES firmware image.

Builds the structure src/Storage/EmbeddedFiles.cpp expects to find at _firmware_end:

    uint32 magic = 0x543C2BEF
    uint32 directoriesOffset
    uint32 numFiles
    { uint32 nameOffset, uint32 contentOffset, uint32 contentLength } * numFiles
    ... names, directory list, file contents

All offsets are relative to _firmware_end, which for a correctly linked image is the current end of
the .bin. Names are full paths with a leading slash and no drive number ("/sys/config.g"), matching
what SkipDriveNumber leaves behind. The directory list is NUL-terminated strings followed by an empty
one.

The image is linked with _firmware_crc == _firmware_end, so vector table slot 7 initially points at
where the filesystem starts. Since AppMain CRCs everything below the address in that slot, the slot
has to be moved past the filesystem - which this does. Run CrcAppender.py afterwards, not before.

Usage: BuildEmbeddedFiles.py <firmware.bin> <rootdir> [--load-address 0x400000]
"""

import argparse
import os
import struct
import sys

MAGIC = 0x543C2BEF
VECTOR_SLOT_FIRMWARE_CRC = 0x1C
HEADER_SIZE = 12
DESCRIPTOR_SIZE = 12


def collect(root):
    """Return [(stored_path, content_bytes)] and the set of directories, both sorted for determinism."""
    files = []
    directories = set()
    for dirpath, _, filenames in os.walk(root):
        rel_dir = os.path.relpath(dirpath, root)
        for name in sorted(filenames):
            if name.startswith('.'):
                continue                                # skip .DS_Store and friends
            parts = [] if rel_dir == '.' else rel_dir.split(os.sep)
            stored = '/' + '/'.join(parts + [name])
            with open(os.path.join(dirpath, name), 'rb') as f:
                files.append((stored, f.read()))
            for i in range(len(parts)):
                directories.add('/' + '/'.join(parts[:i + 1]))
    files.sort(key=lambda item: item[0])
    return files, sorted(directories)


def build(files, directories):
    body = bytearray()
    body_base = HEADER_SIZE + DESCRIPTOR_SIZE * len(files)

    name_offsets = []
    for stored, _ in files:
        name_offsets.append(body_base + len(body))
        body += stored.encode('ascii') + b'\0'

    directories_offset = body_base + len(body)
    for directory in directories:
        body += directory.encode('ascii') + b'\0'
    body += b'\0'                                       # empty string terminates the list

    descriptors = bytearray()
    for (stored, content), name_offset in zip(files, name_offsets):
        content_offset = body_base + len(body)
        body += content
        descriptors += struct.pack('<III', name_offset, content_offset, len(content))

    return struct.pack('<III', MAGIC, directories_offset, len(files)) + bytes(descriptors) + bytes(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('binary')
    parser.add_argument('rootdir', help='directory whose contents become the filesystem, e.g. one containing sys/')
    parser.add_argument('--load-address', default='0x400000')
    args = parser.parse_args()

    load_address = int(args.load_address, 0)

    with open(args.binary, 'rb') as f:
        image = bytearray(f.read())

    crc_address, = struct.unpack_from('<I', image, VECTOR_SLOT_FIRMWARE_CRC)
    if crc_address - load_address != len(image):
        raise SystemExit(
            f"{args.binary}: vector slot 7 is {crc_address:#x}, which is not the end of a {len(image):#x}-byte image. "
            "Either this is not a USE_EMBEDDED_FILES build, or a filesystem or CRC has already been appended.")

    files, directories = collect(args.rootdir)
    if not files:
        raise SystemExit(f"{args.rootdir}: no files found")

    filesystem = build(files, directories)
    image += filesystem

    # Move the CRC past the filesystem so that AppMain covers firmware and files both.
    struct.pack_into('<I', image, VECTOR_SLOT_FIRMWARE_CRC, load_address + len(image))

    with open(args.binary, 'wb') as f:
        f.write(image)

    for stored, content in files:
        print(f"  {stored}  {len(content)} bytes", file=sys.stderr)
    print(f"{args.binary}: {len(files)} file(s), {len(filesystem)} bytes of filesystem, "
          f"CRC now expected at {load_address + len(image):#x}", file=sys.stderr)


if __name__ == '__main__':
    main()

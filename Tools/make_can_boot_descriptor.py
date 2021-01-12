#!/usr/bin/env python3

import os
import sys
import subprocess
import struct
import optparse
import binascii
from io import BytesIO

class GitWrapper:
    @classmethod
    def command(cls, txt):
        cmd = "git " + txt
        pr = subprocess.Popen( cmd , shell = True, stdout = subprocess.PIPE, stderr = subprocess.PIPE )
        (out, error) = pr.communicate()
        if len(error):
            raise Exception(cmd +" failed with [" + error.strip() + "]")
        return out

class AppDescriptor(object):
    """
    UAVCAN firmware image descriptor format:
    uint64_t signature (bytes [7:0] set to 'APDesc00' by linker script)
    uint64_t image_crc (set to 0 by linker script)
    uint32_t image_size (set to 0 by linker script)
    uint32_t vcs_commit (set in source or by this tool)
    uint8_t version_major (set in source)
    uint8_t version_minor (set in source)
    uint8_t reserved[6] (set to 0xFF by linker script)
    """

    LENGTH = 8 + 8 + 4 + 4 + 1 + 1 + 6
    SIGNATURE = b"APDesc00"
    RESERVED =  b"\xFF" * 6

    def __init__(self, bytes=None):
        self.signature = AppDescriptor.SIGNATURE
        self.image_crc = 0
        self.image_size = 0
        self.vcs_commit = 0
        self.version_major = 0
        self.version_minor = 0
        self.reserved = AppDescriptor.RESERVED

        if bytes:
            try:
                self.unpack(bytes)
            except Exception:
                raise ValueError("Invalid AppDescriptor: {0}".format(
                                 binascii.b2a_hex(bytes)))

    def pack(self):
        return struct.pack("<8sQLLBB6s", self.signature, self.image_crc,
                           self.image_size, self.vcs_commit,
                           self.version_major, self.version_minor,
                           self.reserved)

    def unpack(self, bytes):
        (self.signature, self.image_crc, self.image_size, self.vcs_commit,
            self.version_major, self.version_minor, self.reserved) = \
            struct.unpack("<8sQLLBB6s", bytes)

        if not self.empty and not self.valid:
            raise ValueError()

    @property
    def empty(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.image_crc == 0 and self.image_size == 0 and
                self.reserved == AppDescriptor.RESERVED)

    @property
    def valid(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.image_crc != 0 and self.image_size > 0 and
                self.reserved == AppDescriptor.RESERVED)


class FirmwareImage(object):
    def __init__(self, path_or_file, mode="r"):
        if getattr(path_or_file, "read", None):
            self._file = path_or_file
            self._do_close = False
            self._padding = 0
        else:
            if "b" not in mode:
                self._file = open(path_or_file, mode + "b")
            else:
                self._file = open(path_or_file, mode)
            self._do_close = True
            self._padding = 4

        if "r" in mode:
            self._contents = BytesIO(self._file.read())
        else:
            self._contents = BytesIO()
        self._do_write = False

        self._length = None
        self._descriptor_offset = None
        self._descriptor_bytes = None
        self._descriptor = None

    def __enter__(self):
        return self

    def __getattr__(self, attr):
        if attr == "write":
            self._do_write = True
        return getattr(self._contents, attr)

    def __iter__(self):
        return iter(self._contents)

    def __exit__(self, *args):
        if self._do_write:
            if getattr(self._file, "seek", None):
                self._file.seek(0)
            self._file.write(self._contents.getvalue())
            if  self._padding:
                self._file.write(b'\xff' * self._padding)

        if self._do_close:
            self._file.close()

    def _write_descriptor_raw(self):
        # Seek to the appropriate location, write the serialized
        # descriptor, and seek back.
        prev_offset = self._contents.tell()
        self._contents.seek(self._descriptor_offset)
        self._contents.write(self._descriptor.pack())
        self._contents.seek(prev_offset)

    def write_descriptor(self):
        # Set the descriptor's length and CRC to the values required for
        # CRC computation
        self.app_descriptor.image_size = self.length
        self.app_descriptor.image_crc = 0

        self._write_descriptor_raw()

        # Update the descriptor's CRC based on the computed value and write
        # it out again
        self.app_descriptor.image_crc = self.crc

        self._write_descriptor_raw()

    @property
    def crc(self):
        MASK = 0xFFFFFFFFFFFFFFFF
        POLY = 0x42F0E1EBA9EA3693

        # Calculate the image CRC with the image_crc field in the app
        # descriptor zeroed out.
        crc_offset = self.app_descriptor_offset + len(AppDescriptor.SIGNATURE)
        content = bytearray(self._contents.getvalue())
        content[crc_offset:crc_offset + 8] = bytearray.fromhex("00" * 8)
        if  self._padding:
            content += bytearray.fromhex("ff" * self._padding)
        val = MASK
        for byte in content:
            val ^= (byte << 56) & MASK
            for bit in range(8):
                if val & (1 << 63):
                    val = ((val << 1) & MASK) ^ POLY
                else:
                    val <<= 1

        return (val & MASK) ^ MASK

    @property
    def padding(self):
        return self._padding

    @property
    def length(self):
        if not self._length:
            # Find the length of the file by seeking to the end and getting
            # the offset
            prev_offset = self._contents.tell()
            self._contents.seek(0, os.SEEK_END)
            self._length = self._contents.tell()
            if self._padding:
                fill = self._length % self._padding
                if fill:
                    self._length += fill
                self._padding = fill
            self._contents.seek(prev_offset)

        return self._length

    @property
    def app_descriptor_offset(self):
        if not self._descriptor_offset:
            # Save the current position
            prev_offset = self._contents.tell()
            # Check each byte in the file to see if a valid descriptor starts
            # at that location. Slow, but not slow enough to matter.
            offset = 0
            while offset < self.length - AppDescriptor.LENGTH:
                self._contents.seek(offset)
                try:
                    # If this throws an exception, there isn't a valid
                    # descriptor at this offset
                    AppDescriptor(self._contents.read(AppDescriptor.LENGTH))
                except Exception:
                    offset += 1
                else:
                    self._descriptor_offset = offset
                    break
            # Go back to the previous position
            self._contents.seek(prev_offset)
            if not self._descriptor_offset:
                raise Exception('AppDescriptor not found')

        return self._descriptor_offset

    @property
    def app_descriptor(self):
        if not self._descriptor:
            # Save the current position
            prev_offset = self._contents.tell()
            # Jump to the descriptor adn parse it
            self._contents.seek(self.app_descriptor_offset)
            self._descriptor_bytes = self._contents.read(AppDescriptor.LENGTH)
            self._descriptor = AppDescriptor(self._descriptor_bytes)
            # Go back to the previous offset
            self._contents.seek(prev_offset)

        return self._descriptor

    @app_descriptor.setter
    def app_descriptor(self, value):
        self._descriptor = value


if __name__ == "__main__":
    parser = optparse.OptionParser(usage="usage: %prog [options] [IN OUT]")
    parser.add_option("--vcs-commit", dest="vcs_commit", default=None,
                      help="set the descriptor's VCS commit value to COMMIT",
                      metavar="COMMIT")
    parser.add_option("-g", "--use-git-hash", dest="use_git_hash", action="store_true",
                      help="set the descriptor's VCS commit value to the current git hash",
                      metavar="GIT")
    parser.add_option("--bootloader-size", dest="bootloader_size", default=0,
                      help="don't write the first SIZE bytes of the image",
                      metavar="SIZE")
    parser.add_option("--bootloader-image", dest="bootloader_image", default=0,
                      help="prepend a bootloader image to the output file",
                      metavar="IMAGE")
    parser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                      help="show additional firmware information on stdout")

    options, args = parser.parse_args()
    if len(args) not in (0, 2):
        parser.error("specify both IN or OUT for file operation, or " +
                     "neither for stdin/stdout operation")

    if options.vcs_commit and options.use_git_hash:
        parser.error("options --vcs-commit and --use-git-commit are mutually exclusive")

    if options.use_git_hash:
        try:
            options.vcs_commit = int(GitWrapper.command("rev-list HEAD --max-count=1 --abbrev=8 --abbrev-commit"),16)
        except Exception  as e:
            print("Git Command failed "+ str(e) +"- Exiting!")
            quit()

    if args:
        in_file = args[0]
        out_file = args[1]
    else:
        in_file = sys.stdin
        out_file = sys.stdout

    bootloader_image = b""
    if options.bootloader_image:
        with open(options.bootloader_image, "rb") as bootloader:
            bootloader_image = bootloader.read()

    bootloader_size = int(options.bootloader_size)

    with FirmwareImage(in_file, "rb") as in_image:
        with FirmwareImage(out_file, "wb") as out_image:
            image = in_image.read()
            out_image.write(bootloader_image)
            out_image.write(image[bootloader_size:])
            if options.vcs_commit:
                out_image.app_descriptor.vcs_commit = options.vcs_commit
            out_image.write_descriptor()

            if options.verbose:
                sys.stderr.write(
"""
Application descriptor located at offset 0x{0.app_descriptor_offset:08X}

""".format(in_image, in_image.app_descriptor, out_image.app_descriptor,
           bootloader_size, len(bootloader_image)))
                if bootloader_size:
                    sys.stderr.write(
"""Ignored the first {3:d} bytes of the input image. Prepended {4:d} bytes of
bootloader image to the output image.

""".format(in_image, in_image.app_descriptor, out_image.app_descriptor,
           bootloader_size, len(bootloader_image)))
                sys.stderr.write(
"""READ VALUES
------------------------------------------------------------------------------

Field               Type              Value
signature           uint64            {1.signature!r}
image_crc           uint64            0x{1.image_crc:016X}
image_size          uint32            0x{1.image_size:X} ({1.image_size:d} B)
vcs_commit          uint32            {1.vcs_commit:08X}
version_major       uint8             {1.version_major:d}
version_minor       uint8             {1.version_minor:d}
reserved            uint8[6]          {1.reserved!r}

WRITTEN VALUES
------------------------------------------------------------------------------

Field               Type              Value
signature           uint64            {2.signature!r}
image_crc           uint64            0x{2.image_crc:016X}
image_size          uint32            0x{2.image_size:X} ({2.image_size:d} B)
vcs_commit          uint32            {2.vcs_commit:08X}
version_major       uint8             {2.version_major:d}
version_minor       uint8             {2.version_minor:d}
reserved            uint8[6]          {2.reserved!r}

""".format(in_image, in_image.app_descriptor, out_image.app_descriptor,
           bootloader_size, len(bootloader_image)))
                if out_image.padding:
                    sys.stderr.write(
"""
padding added {}
""".format(out_image.padding))

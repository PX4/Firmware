#!/usr/bin/env python3

import os
import sys
import subprocess
import struct
import optparse
import binascii
from io import BytesIO
import array

crctab = array.array('I', [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d])

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
    uint64_t signature (bytes [7:0] set to '{ 0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06 }' by the build
    uint32_t crc32_block1   From offset 0 to . (non inclusive) (set by this tool)
    uint32_t crc32_block2   From offsetof(minor_version) to end  (set by this tool)
    uint32_t image_size    (set to 0 by linker script)
    uint32_t vcs_commit    (set in source or by this tool)
    uint8_t version_major (set in source)
    uint8_t version_minor (set in source)
    uint16_t board_id     (set in source)
    uint8_t reserved[6] (set to 0xFF by linker script)
    """

    LENGTH = 8 + 4 + 4 + 4 + 4 + 1 + 1 + 2 + 8
    DESLENGTH = 4 + 4 + 4 + 4
    SIGNATURE = b"\x40\xa2\xe4\xf1\x64\x68\x91\x06"
    RESERVED =  b"\xFF" * 8

    def __init__(self, bytes=None):
        self.signature = AppDescriptor.SIGNATURE
        self.crc32_block1 = 0
        self.crc32_block2 = 0
        self.image_size = 0
        self.vcs_commit = 0
        self.version_major = 0
        self.version_minor = 0
        self.board_id = 0
        self.reserved = AppDescriptor.RESERVED

        if bytes:
            try:
                self.unpack(bytes)
            except Exception:
                raise ValueError("Invalid AppDescriptor: {0}".format(
                                 binascii.b2a_hex(bytes)))

    def pack(self):
        return struct.pack("<8sLLLLBBH8s", self.signature, self.crc32_block1,
                           self.crc32_block2, self.image_size, self.vcs_commit,
                           self.version_major, self.version_minor,
                           self.board_id,
                           self.reserved)

    def unpack(self, bytes):
        (self.signature, self.crc32_block1, self.crc32_block2, self.image_size, self.vcs_commit,
            self.version_major, self.version_minor, self.board_id, self.reserved) = \
            struct.unpack("<8sLLLLBBH8s", bytes)

        if not self.empty and not self.valid:
            raise ValueError()

    @property
    def empty(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.crc32_block1 == 0  and self.crc32_block2 == 0 and
                self.image_size == 0 and self.reserved == AppDescriptor.RESERVED)

    @property
    def valid(self):
        return (self.signature == AppDescriptor.SIGNATURE and
                self.crc32_block1 != 0  and self.crc32_block2 != 0 and
                self.image_size > 0 and self.board_id != 0 and
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
        self.app_descriptor.crc32_block1 = 0
        self.app_descriptor.crc32_block2 = 0

        self._write_descriptor_raw()

        content = bytearray(self._contents.getvalue())
        if  self._padding:
            content += bytearray.fromhex("ff" * self._padding)

        # Update the descriptor's CRC based on the computed value and write
        # it out again

        self.app_descriptor.crc32_block1 = self.crc32(content[:self.app_descriptor_offset + len(AppDescriptor.SIGNATURE)])
        b2 = self.app_descriptor_offset + len(AppDescriptor.SIGNATURE) + AppDescriptor.DESLENGTH
        self.app_descriptor.crc32_block2 = self.crc32(content[b2:])

        self._write_descriptor_raw()

    def crc32(self, bytes, crc = 0):


        for byte in bytes:
            index = (crc ^ byte) & 0xff
            crc = crctab[index] ^ (crc >> 8)
        return crc

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
                fill = self._padding - (self._length % self._padding)
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
crc32_block1        uint32            0x{1.crc32_block1:08X}
crc32_block2        uint32            0x{1.crc32_block2:08X}
image_size          uint32            0x{1.image_size:X} ({1.image_size:d} B)
vcs_commit          uint32            {1.vcs_commit:08X}
version_major       uint8             {1.version_major:d}
version_minor       uint8             {1.version_minor:d}
board_id            uint32            0x{1.board_id:X}
reserved            uint8[8]          {1.reserved!r}

WRITTEN VALUES
------------------------------------------------------------------------------
Field               Type              Value
signature           uint64            {2.signature!r}
crc32_block1        uint32            0x{2.crc32_block1:08X}
crc32_block2        uint32            0x{2.crc32_block2:08X}
image_size          uint32            0x{2.image_size:X} ({2.image_size:d} B)
vcs_commit          uint32            {2.vcs_commit:08X}
version_major       uint8             {2.version_major:d}
version_minor       uint8             {2.version_minor:d}
board_id            uint32            0x{2.board_id:X}
reserved            uint8[8]          {2.reserved!r}
""".format(in_image, in_image.app_descriptor, out_image.app_descriptor,
           bootloader_size, len(bootloader_image)))
                if out_image.padding:
                    sys.stderr.write(
"""
padding added {}
""".format(out_image.padding))

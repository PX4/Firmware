#!/usr/bin/env python3

from Crypto.PublicKey import RSA
from Crypto.Cipher import PKCS1_OAEP
from Crypto.Cipher import ChaCha20
from Crypto.Hash import SHA256
import binascii
import argparse
#from pathlib import Path
import sys

if(__name__ == "__main__"):

    parser = argparse.ArgumentParser(description="""CLI tool to decrypt an ulog file\n""")
    parser.add_argument("ulog_file", help=".ulog file", nargs='?', default=None)
    parser.add_argument("ulog_key", help=".ulogk, encrypted key", nargs='?', default=None)
    parser.add_argument("rsa_key", help=".pem format key for decrypting the ulog key", nargs='?', default=None)

    args = parser.parse_args()

    # Only generate a key pair, don't sign
    if not args.ulog_file or not args.ulog_key or not args.rsa_key:
        print('Need all arguments, the encrypted ulog file, the key and the key decryption key')
        sys.exit(1);

    # Read the private RSA key to decrypt the cahcha key
    with open(args.rsa_key, 'rb') as f:
        r = RSA.importKey(f.read(), passphrase='')

    # Read the encrypted xchacha key and the nonce
    with open(args.ulog_key, 'rb') as f:
        ulog_key_cipher = f.read(256)
        nonce = f.read(24)

    # Decrypt the xchacha key
    cipher_rsa = PKCS1_OAEP.new(r,SHA256)
    ulog_key = cipher_rsa.decrypt(ulog_key_cipher)
    #print(binascii.hexlify(ulog_key))

    # Read and decrypt the .ulgc
    cipher = ChaCha20.new(key=ulog_key, nonce=nonce)
    with open(args.ulog_file, 'rb') as f:
        with open(args.ulog_file.rstrip(args.ulog_file[-1]), 'wb') as out:
            out.write(cipher.decrypt(f.read()))

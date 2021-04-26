#!/usr/bin/env python3

import nacl.encoding
import nacl.signing
import nacl.hash
import struct
import binascii
import json
import time
import argparse
from pathlib import Path
import sys

def make_public_key_h_file(signing_key,key_name):
    """
    This file generate the public key header file
    to be included into the bootloader build.
    """
    public_key_c='\n'
    for i,c in enumerate(signing_key.verify_key.encode(encoder=nacl.encoding.RawEncoder)):
        public_key_c+= hex(c)
        public_key_c+= ', '
        if((i+1)%8==0):
            public_key_c+= '\n'
    with open(key_name+'.pub' ,mode='w') as f:
        f.write("//Public key to verify signed binaries")
        f.write(public_key_c)

def make_key_file(signing_key, key_name):
    """
    Writes the key.json file.
    Attention do not override your existing key files.
    Do not publish your private key!!
    """

    key_file = Path(key_name+'.json')
    if key_file.is_file():
        print("ATTENTION: key.json already exists, are you sure you want to overwrite it?")
        print("Remove file and run script again.")
        print("Script aborted!")
        sys.exit(1)

    keys={}
    keys["date"] = time.asctime()
    keys["public"] = (signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder)).decode()
    keys["private"] = binascii.hexlify(signing_key._seed).decode()
    #print (keys)
    with open(key_name+'.json', "w") as write_file:
        json.dump(keys, write_file)
    return keys

def ed25519_sign(private_key, signee_bin):
    """
    This function creates the signature. It takes the private key and the binary file
    and returns the tuple (signature, public key)
    """

    signing_key = nacl.signing.SigningKey(private_key, encoder=nacl.encoding.HexEncoder)

    # Sign a message with the signing key
    signed = signing_key.sign(signee_bin,encoder=nacl.encoding.RawEncoder)

    # Obtain the verify key for a given signing key
    verify_key = signing_key.verify_key

    # Serialize the verify key to send it to a third party
    verify_key_hex = verify_key.encode(encoder=nacl.encoding.HexEncoder)

    return signed.signature, verify_key_hex


def sign(bin_file_path, key_file_path=None, generated_key_file=None):
    """
    reads the binary file and the key file.
    If the key file does not exist, it generates a
    new key file.
    """

    with open(bin_file_path,mode='rb') as f:
        signee_bin = f.read()
        # Align to 4 bytes. Signature always starts at
        # 4 byte aligned address, but the signee size
        # might not be aligned
        signee_bin += bytearray(b'\xff')*(4-len(signee_bin)%4)

    try:
        with open(key_file_path,mode='r') as f:
            keys = json.load(f)
        #print(keys)
    except:
        print('ERROR: Key file',key_file_path,'not  found')
        sys.exit(1)

    signature, public_key = ed25519_sign(keys["private"], signee_bin)

    # Do a sanity check. This type of signature is always 64 bytes long
    assert len(signature) == 64

    # Print out the signing information
    print("Binary \"%s\" signed."%bin_file_path)
    print("Signature:",binascii.hexlify(signature))
    print("Public key:",binascii.hexlify(public_key))

    return signee_bin + signature, public_key

def generate_key(key_file):
    """
    Generate two files:
      "key_file.pub" containing the public key in C-format to be included in the bootloader build
      "key_file.json, containt both private and public key.
    Do not leak or loose the key file. This is mandatory for signing
    all future binaries you want to deploy!
    """

    # Generate a new random signing key
    signing_key = nacl.signing.SigningKey.generate()
     # Serialize the verify key to send it to a third party
    verify_key_hex = signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder)
    print("public key :",verify_key_hex)

    private_key_hex=binascii.hexlify(signing_key._seed)
    print("private key :",private_key_hex)

    keys = make_key_file(signing_key,key_file)
    make_public_key_h_file(signing_key,key_file)
    return keys

if(__name__ == "__main__"):

    parser = argparse.ArgumentParser(description="""CLI tool to calculate and add signature to px4. bin files\n
                                                  if given it takes an existing key file, else it generate new keys""",
                                    epilog="Output: SignedBin.bin and a key.json file")
    parser.add_argument("signee", help=".bin file to add signature", nargs='?', default=None)
    parser.add_argument("signed", help="signed output .bin", nargs='?', default=None)

    parser.add_argument("--key", help="key.json file", default="Tools/test_keys.json")
    parser.add_argument("--rdct", help="binary R&D certificate file", default=None)
    parser.add_argument("--genkey", help="new generated key", default=None)
    args = parser.parse_args()

    # Only generate a key pair, don't sign
    if args.genkey:
        # Only create a key file, don't sign
        generate_key(args.genkey)
        print('New key file generated:',args.genkey)
        sys.exit(0);

    # Check that both signee and signed exist
    if not args.signee or not args.signed:
        print("ERROR: Must either provide file names for both signee and signed")
        print("       or --genkey [key] to generate a new key pair")
        sys.exit(1)

    # Issue a warning when signing with testing key
    if args.key=='Tools/test_keys.json':
        print("WARNING: Signing with PX4 test key")

    # Sign the binary
    signed, public_key = sign(args.signee, args.key, args.genkey)

    with open(args.signed, mode='wb') as fs:
        # Write signed binary
        fs.write(signed)

    # Append rdcert if given
    try:
        with open(args.rdct ,mode='rb') as f:
            with open(args.signed, mode='ab') as fs:
                fs.write(f.read())
    except:
        pass


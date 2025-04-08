"""Calculator for MIPI debugging
"""
from typing import List
import numpy as np

from cobra_system_control.utility import (
    ptob_raw12, btop_raw12, ptob_raw16, btop_raw16,
)


def mipi_crc16(byte_list: List[int]):
    """byte_list excludes header (see mipi_ecc)
    """
    # See section 9.6 of MIPI CSI-2 spec
    # POLY is the bit-reversal of 0x1021 -> x**16 == x**12 + x**5 + x**0
    POLY = 0x8408

    crc = 0xffff
    for b in byte_list:
        data = b
        for i in range(8):
            if (crc ^ data) & 0x0001 == 1:
                crc = (crc >> 1) ^ POLY
            else:
                crc >>= 1
            data >>= 1
            #print(hex(crc))
        #print('*')
    return crc


def mipi_ecc(vc: int, dt: int, wc: int, vcx: int = 0):
    """
    """
    # See section 9.5 of MIPI CSI-2 spec:
    #      HEADER[25:24] = VCX = 2'b00
    #      HEADER[23:16] = WC[15:8]
    #      HEADER[15:08] = WC[7:0]
    #      HEADER[07:00] = DI[7:0] = {VC[1:0], DT[5:0]}
    d_int = (
        (vcx & 0x3) << 24 |
        (wc & 0xffff) << 8 |
        (vc & 0x3) << 6 |
        (dt & 0x3f) << 0
    )
    d = [int(b, 2) for b in reversed('{:026b}'.format(d_int))]

    # Adapted from Table 12 of MIPI CSI-2 Spec:
    #      i    P5     P4      P3      P2      P1      P0      Hex
    #      ------------------------------------------------------------
    #      0    1'b0   1'b0    1'b0    d[0]    d[0]    d[0]    0x07
    #      1    1'b0   1'b0    d[1]    1'b0    d[1]    d[1]    0x0B
    #      2    1'b0   1'b0    d[2]    d[2]    1'b0    d[2]    0x0D
    #      3    1'b0   1'b0    d[3]    d[3]    d[3]    1'b0    0x0E
    #      4    1'b0   d[4]    1'b0    1'b0    d[4]    d[4]    0x13
    #      5    1'b0   d[5]    1'b0    d[5]    1'b0    d[5]    0x15
    #      6    1'b0   d[6]    1'b0    d[6]    d[6]    1'b0    0x16
    #      7    1'b0   d[7]    d[7]    1'b0    1'b0    d[7]    0x19
    #      8    1'b0   d[8]    d[8]    1'b0    d[8]    1'b0    0x1A
    #      9    1'b0   d[9]    d[9]    d[9]    1'b0    1'b0    0x1C
    #      10   d[10]  1'b0    1'b0    1'b0    d[10]   d[10]   0x23
    #      11   d[11]  1'b0    1'b0    d[11]   1'b0    d[11]   0x25
    #      12   d[12]  1'b0    1'b0    d[12]   d[12]   1'b0    0x26
    #      13   d[13]  1'b0    d[13]   1'b0    1'b0    d[13]   0x29
    #      14   d[14]  1'b0    d[14]   1'b0    d[14]   1'b0    0x2A
    #      15   d[15]  1'b0    d[15]   d[15]   1'b0    1'b0    0x2C
    #      16   d[16]  d[16]   1'b0    1'b0    1'b0    d[16]   0x31
    #      17   d[17]  d[17]   1'b0    1'b0    d[17]   1'b0    0x32
    #      18   d[18]  d[18]   1'b0    d[18]   1'b0    1'b0    0x34
    #      19   d[19]  d[19]   d[19]   1'b0    1'b0    1'b0    0x38
    #      20   1'b0   d[20]   d[20]   d[20]   d[20]   d[20]   0x1F
    #      21   d[21]  1'b0    d[21]   d[21]   d[21]   d[21]   0x2F
    #      22   d[22]  d[22]   1'b0    d[22]   d[22]   d[22]   0x37
    #      23   d[23]  d[23]   d[23]   1'b0    d[23]   d[23]   0x3B
    #      24   d[24]  d[24]   d[24]   d[24]   1'b0    d[24]   0x3D
    #      25   d[25]  d[25]   d[25]   d[25]   d[25]   1'b0    0x3E
    ecc = [
        d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23]^d[24]^d[25],  # 5
        d[ 4]^d[ 5]^d[ 6]^d[ 7]^d[ 8]^d[ 9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23]^d[24]^d[25],  # 4
        d[ 1]^d[ 2]^d[ 3]^d[ 7]^d[ 8]^d[ 9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23]^d[24]^d[25],  # 3
        d[ 0]^d[ 2]^d[ 3]^d[ 5]^d[ 6]^d[ 9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22]^d[24]^d[25],  # 2
        d[ 0]^d[ 1]^d[ 3]^d[ 4]^d[ 6]^d[ 8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23]^d[25],  # 1
        d[ 0]^d[ 1]^d[ 2]^d[ 4]^d[ 5]^d[ 7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23]^d[24],  # 0
    ]
    ecc_int = 0
    for i, b in enumerate(reversed(ecc)):
        ecc_int |= b << i
    return ecc_int


def test():
    # CRC
    vec1 = [
        # L0  Lane1 Lane2 Lane3
        0xFF, 0x00, 0x00, 0x02,
        0xB9, 0xDC, 0xF3, 0x72,
        0xBB, 0xD4, 0xB8, 0x5A,
        0xC8, 0x75, 0xC2, 0x7C,
        0x81, 0xF8, 0x05, 0xDF,
        0xFF, 0x00, 0x00, 0x01,
        #CRCL CRCH  ----  ----
    ]
    res = mipi_crc16(vec1)
    print(f'res = 0x{res:04x}')
    assert res == 0x00f0

    # CRC
    vec2 = [
        # L0  Lane1 Lane2 Lane3
        0xFF, 0x00, 0x00, 0x00,
        0x1E, 0xF0, 0x1E, 0xC7,
        0x4F, 0x82, 0x78, 0xC5,
        0x82, 0xE0, 0x8C, 0x70,
        0xD2, 0x3C, 0x78, 0xE9,
        0xFF, 0x00, 0x00, 0x01,
        #CRCL CRCH  ----  ----
    ]
    res = mipi_crc16(vec2)
    print(f'res = 0x{res:04x}')
    assert res == 0xe569

    # ECC
    res = mipi_ecc(0, 0x37, 0x1f0, 1)
    print(f'res = 0b{res:06b} = 0x{res:02x}')
    assert res == 2


def quantize(v, quant_mode):
    """Quantization performed per quant_mode when reduce_mode == 1."""
    return np.clip(np.floor(v / 2**quant_mode), 0, 2**12 - 1).astype(int)


def main():
    raw16_en = 1  # 1: RAW8/RAW16, 0: RAW12
    swap_bytes = 1  # 1: RAW8, 0: RAW16
    reduce_mode = 1
    quant_mode = 1

    n_pixels = 640
    vc = 0
    dt = 0x2c if not raw16_en else 0x2e if not swap_bytes else 0x2a
    wc = n_pixels * 3 * (16 if raw16_en else 12) // 8
    ecc = mipi_ecc(vc, dt, wc)

    # test_mode = 3 (tap mode).
    pixels = []
    for pixel in range(n_pixels):
        if reduce_mode == 0:
            tap_a = (3 * pixel + 0)
            tap_b = (3 * pixel + 1)
            tap_c = (3 * pixel + 2)
        elif raw16_en == 0:
            tap_a = quantize(9 * pixel + 3, quant_mode)
            tap_b = tap_a
            tap_c = tap_a
        else:
            tap_a = (9 * pixel + 3)
            tap_b = tap_a
            tap_c = tap_a
        pixels.extend((tap_a, tap_b, tap_c))

    if raw16_en:
        bytes_ = ptob_raw16(pixels, swap_bytes=swap_bytes)
        assert btop_raw16(bytes_, swap_bytes=swap_bytes) == pixels
    else:
        bytes_ = ptob_raw12(pixels)
        assert btop_raw12(bytes_) == pixels
    assert len(bytes_) == wc
    crc = mipi_crc16(bytes_)

    bytes_ = [
        vc << 6 | dt,
        wc & 0xff,
        wc >> 8,
        ecc,
        *bytes_,
        crc & 0xff,
        crc >> 8,
    ]

    #print_pixels = slice(None)
    #print_bytes = slice(None)
    print_pixels = slice(0, 33)
    print_bytes = slice(0, 64)

    print()
    print(f"raw16_en    = {raw16_en}")
    print(f"swap_bytes  = {swap_bytes}")
    print(f"test_mode   = 3")
    print(f"reduce_mode = {reduce_mode}")
    print(f"quant_mode  = {quant_mode}")
    print()

    print("Pixels  A,   B,   C")
    for i, b in enumerate(pixels[print_pixels]):
        print(f'{i//3: 4d}  ' if i % 3 == 0 else '', end='')
        print(f'{b:0{4 if raw16_en else 3}x}  ', end=('\n' if i % 3 == 2 else ''))
    print()

    print("Bytes L0, L1, L2, L3")
    for i, b in enumerate(bytes_[print_bytes]):
        print(f'{i//4: 4d}  ' if i % 4 == 0 else '', end='')
        print(f'{b:02x}  ', end=('\n' if i % 4 == 3 else ''))
    print()

    print("uint16  A,   B,   C")
    for i, b in enumerate(pixels[print_pixels]):
        # Values are left-justified. See:

        sh = 2 if raw16_en else 4
        width = 4 if raw16_en else 3
        print(f'{i//3: 4d}  ' if i % 3 == 0 else '', end='')
        print(f'{b << sh:0{width}x}  ', end=('\n' if i % 3 == 2 else ''))
    print()


if __name__ == "__main__":
    test()
    main()

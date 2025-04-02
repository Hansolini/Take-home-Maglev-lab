
'''
    A python library for COBS encoding/decoding does already exist!!!
'''

# COBS (Consistent Overhead Byte Stuffing) encoding
def cobs_encode(data: bytes) -> bytes:
    encoded = bytearray()
    code_ptr = 0  # position of the code byte
    code = 1      # initial code value
    encoded.append(0)  # placeholder for the first code byte

    for byte in data:
        if byte == 0:  # zero byte found, close the block
            encoded[code_ptr] = code
            code_ptr = len(encoded)
            encoded.append(0)  # new code byte placeholder
            code = 1
        else:
            encoded.append(byte)
            code += 1
            if code == 0xFF:  # max block size reached
                encoded[code_ptr] = code
                code_ptr = len(encoded)
                encoded.append(0)
                code = 1

    encoded[code_ptr] = code  # final block code byte
    encoded.append(0)  # end marker
    return bytes(encoded)


# COBS decoding
def cobs_decode(data: bytes) -> bytes:
    decoded = bytearray()
    ptr = 0


    while ptr < len(data) - 1:  # ignore the final zero marker
        code = data[ptr]
        ptr += 1
        for i in range(code - 1):
            decoded.append(data[ptr])
            ptr += 1
        if code < 0xFF:  # only add a zero if code < 255
            decoded.append(0)

    return bytes(decoded[:-1])  # ignore the last \x00

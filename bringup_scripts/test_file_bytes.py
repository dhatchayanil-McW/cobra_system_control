import pickle
import numpy as np


if __name__ == "__main__":
    a = list(range(256))
    ba = bytearray(a)
    #print(a, ba, ba[0], bytes(a))
    fid = './byte_test.bin'
    with open(fid, 'wb') as f:
        f.write(ba)

    pfid = './byte_test.bin'
    b = np.asarray(a, dtype=np.float32)
    with open(pfid, 'wb') as f:
        pickle.dump(bytearray(b), f, protocol=pickle.HIGHEST_PROTOCOL)

    with open(pfid, 'rb') as f:
        pdata = pickle.load(f)

    print(b, bytearray(pdata))
    assert bytearray(b) == bytearray(pdata)

    with open(fid, 'rb') as f:
        rdata = f.readline()

    d = bytearray()
    #bdata = d+ rdata
    bdata = bytearray(rdata)
    print(len(bdata))
    print(bdata)

    assert bdata == ba

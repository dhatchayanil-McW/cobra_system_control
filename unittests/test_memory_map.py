import os
import random
import pytest as pt

from cobra_system_control.memory_map import MemoryMap


TEST_MAP_YAML_FILE_8 = os.path.join(
    os.path.dirname(__file__),
    'test_memory_map_8.yml',
)
TEST_MAP_YAML_FILE_32 = os.path.join(
    os.path.dirname(__file__),
    'test_memory_map_32.yml',
)


def get_rand_bytes(n_bytes):
    return [random.randint(0, 0xff) for _ in range(n_bytes)]


def get_rand_uints(n_ints, n_bits):
    return [random.randint(0, 2**n_bits - 1) for _ in range(n_ints)]


def get_rand_ints(n_ints, n_bits):
    return [random.randint(-2**(n_bits - 1), 2**(n_bits - 1) - 1)
            for _ in range(n_ints)]


class Memory:
    """Typical usage of MemoryMap. Instead of 'Memory', users might prefer
    to copy this class and use the class name 'FpgaMemory' with a global
    instance named 'fpga' or 'fpgamem'. Users should replace the
    definitions of all methods except __init__().
    """

    def __init__(self, map_yaml, init_mem=0):
        self.memmap = MemoryMap(map_yaml)
        for pname, periph in self.memmap.periphs.items():
            periph.register_read_callback(self.read)
            periph.register_readdata_callback(lambda x: x)
            periph.register_write_callback(self.write)
        self.init_mem = init_mem
        self.mem = {}

    def read(self, addr):
        word_addr = self.b2w_addr(addr)
        if word_addr not in self.mem:
            self.mem[word_addr] = self.init_mem
        return self.mem[word_addr]

    def write(self, addr, data):
        word_addr = self.b2w_addr(addr)
        self.mem[word_addr] = data

    def b2w_addr(self, byte_addr):
        return byte_addr // (self.memmap.word_size // 8)

    def w2b_addr(self, word_addr):
        return word_addr * (self.memmap.word_size // 8)

    def dump_mem(self, prefix=''):
        s = ''
        for word_addr, data in sorted(self.mem.items()):
            addr = self.w2b_addr(word_addr)
            s += f'{prefix}{addr:#011_x}, {data:#011_x}\n'
        return s

    def diff_mem(self, mem):
        word_addrs = set()
        word_addrs.update(mem.keys(), self.mem.keys())
        diff = {}
        for k in word_addrs:
            d = mem.get(k, self.init_mem) ^ self.mem.get(k, self.init_mem)
            if d != 0:
                diff[k] = d
        return diff


@pt.mark.parametrize('map_yaml', [
    pt.param(TEST_MAP_YAML_FILE_32),
    pt.param(TEST_MAP_YAML_FILE_8),
])
@pt.mark.parametrize('periph_name, init_mem_ones', [
    pt.param("periph0", False),
    pt.param("periph1", False),
    pt.param("periph0", True),
    pt.param("periph1", True),
])
def test_memory_map(map_yaml, periph_name, init_mem_ones):
    memory = Memory(map_yaml)

    if init_mem_ones:
        memory.init_mem = memory.memmap.word_mask
    else:
        memory.init_mem = 0

    periph = memory.memmap.periphs[periph_name]
    for field_name in periph.fields.keys():
        f_obj = periph.fields[field_name]
        old_mem = memory.mem.copy()
        wdata = get_rand_uints(1, f_obj.size)[0]
        periph.write_fields(**{field_name: wdata})
        rdata = periph.read_fields(field_name)
        diff = memory.diff_mem(old_mem)

        msg = (f'{periph_name}, {field_name}, wdata = {hex(wdata)}, '
               f'{wdata}, rdata = {hex(rdata)}, {rdata} '
               f'diff = {diff}, dump = {memory.dump_mem(prefix="    ")}')

        assert len(diff) <= f_obj.n_words, msg
        assert wdata == rdata, msg

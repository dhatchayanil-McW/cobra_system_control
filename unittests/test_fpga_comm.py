import random
import pytest as pt


def test_read(mock, fpga_dbg, lcm_ctrl, golden_shas):
    ver_str = ', '.join(golden_shas.keys())

    if mock:
        fpga_dbg.read_fields.return_value = random.choice(list(golden_shas.values()))
        lcm_ctrl.read_fields.return_value = 100

    git_sha = fpga_dbg.read_fields('git_sha')
    msg = f"git sha {git_sha:08x} isn't an approved version: {ver_str}."
    assert git_sha in golden_shas.values(), msg

    clk_freq = lcm_ctrl.read_fields('clk_freq')
    assert clk_freq == 100, f'clk_freq not 100, = {clk_freq}'


def test_read_write(mock, fpga_dbg):
    if mock:
        pt.skip('integration only')
    for _ in range(100):
        aword = random.randint(0, 2**8 - 1)
        fpga_dbg.write_fields(scratch_a=aword)
        bword = random.randint(0, 2**8 - 1)
        fpga_dbg.write_fields(scratch_b=bword)

        adata = fpga_dbg.read_fields('scratch_a')
        bdata = fpga_dbg.read_fields('scratch_b')
        msg = f'word not equal data, word = {aword}, data = {adata}'
        assert aword == adata, msg
        msg = f'word not equal data, word = {bword}, data = {bdata}'
        assert bword == bdata, msg

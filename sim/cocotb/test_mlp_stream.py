import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


def pack16(v):
    return v & 0xFFFF


def sign16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def cpu_mlp(in_vec, w1, b1, w2, b2, dim_in, dim_hid, dim_out, shift):
    hid = [0] * dim_hid
    for h in range(dim_hid):
        acc = b1[h]
        row = w1[h * dim_in : (h + 1) * dim_in]
        for i in range(dim_in):
            acc += row[i] * in_vec[i]
        acc >>= shift
        if acc < 0:
            acc = 0
        hid[h] = acc
    out = [0] * dim_out
    for o in range(dim_out):
        acc = b2[o]
        row = w2[o * dim_hid : (o + 1) * dim_hid]
        for h in range(dim_hid):
            acc += row[h] * hid[h]
        acc >>= shift
        if acc > 32767:
            acc = 32767
        if acc < -32768:
            acc = -32768
        out[o] = acc
    return out


async def wb_mem_model(dut, mem):
    dut.bus_ack.value = 0
    dut.bus_dat_r.value = 0
    while True:
        await RisingEdge(dut.sys_clk)
        if dut.bus_cyc.value and dut.bus_stb.value:
            addr = int(dut.bus_adr.value)
            if dut.bus_we.value:
                data = int(dut.bus_dat_w.value) & 0xFFFFFFFF
                mem[addr] = data
            else:
                data = mem.get(addr, 0)
                dut.bus_dat_r.value = data
            dut.bus_ack.value = 1
        else:
            dut.bus_ack.value = 0


async def pulse_start(dut):
    dut.storage.value = 1
    dut.re.value = 1
    await RisingEdge(dut.sys_clk)
    dut.re.value = 0


@cocotb.test()
async def test_mlp_stream(dut):
    cocotb.start_soon(Clock(dut.sys_clk, 10, units="ns").start())

    dut.sys_rst.value = 1
    await Timer(100, units="ns")
    dut.sys_rst.value = 0

    dim_in = 4
    dim_hid = 8
    dim_out = 2
    shift = 8

    in_vec = [32, -16, 24, 8]
    w1 = []
    b1 = []
    w2 = []
    b2 = []

    for h in range(dim_hid):
        b1.append(h - 4)
        for i in range(dim_in):
            w1.append((h + 1) * (i + 1))

    for o in range(dim_out):
        b2.append(o - 1)
        for h in range(dim_hid):
            w2.append((o + 1) * (h + 1))

    base = 0x0000
    in_off = 0
    w1_off = in_off + dim_in
    b1_off = w1_off + (dim_in * dim_hid)
    hid_off = b1_off + dim_hid
    w2_off = hid_off + dim_hid
    b2_off = w2_off + (dim_out * dim_hid)
    out_off = b2_off + dim_out

    mem = {}
    for i, v in enumerate(in_vec):
        mem[in_off + i] = pack16(v)
    for i, v in enumerate(w1):
        mem[w1_off + i] = pack16(v)
    for i, v in enumerate(b1):
        mem[b1_off + i] = pack16(v)
    for i, v in enumerate(w2):
        mem[w2_off + i] = pack16(v)
    for i, v in enumerate(b2):
        mem[b2_off + i] = pack16(v)

    cocotb.start_soon(wb_mem_model(dut, mem))

    dut.storage_1.value = 1
    dut.storage_2.value = shift
    dut.storage_3.value = dim_in
    dut.storage_4.value = dim_hid
    dut.storage_5.value = dim_out

    dut.storage_6.value = base + (in_off << 2)
    dut.storage_7.value = base + (w1_off << 2)
    dut.storage_8.value = base + (b1_off << 2)
    dut.storage_9.value = base + (w2_off << 2)
    dut.storage_10.value = base + (b2_off << 2)
    dut.storage_11.value = base + (hid_off << 2)
    dut.storage_12.value = base + (out_off << 2)

    await pulse_start(dut)

    for _ in range(200000):
        await RisingEdge(dut.sys_clk)
        if dut.status_1.value:
            break
    else:
        assert False, "MLP did not complete"

    hw_out = [sign16(mem[out_off + i]) for i in range(dim_out)]
    sw_out = cpu_mlp(in_vec, w1, b1, w2, b2, dim_in, dim_hid, dim_out, shift)

    dut._log.info(f"HW out: {hw_out}")
    dut._log.info(f"SW out: {sw_out}")

    assert hw_out == sw_out

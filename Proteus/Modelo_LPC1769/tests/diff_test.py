#!/usr/bin/env python3
"""Prueba diferencial del intérprete ARMv7-M contra Unicorn (motor de QEMU).

Genera bloques de instrucciones Thumb-2 aleatorias, los ensambla con
arm-none-eabi-as/ld, los ejecuta en los dos emuladores desde el mismo estado
inicial y compara registros, flags y RAM.

Uso: diff_test.py <libcpu_api.so> <directorio_toolchain_bin> [cantidad] [semilla]
"""
import ctypes
import os
import random
import subprocess
import sys
import tempfile

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UcError
from unicorn import arm_const as A

CODE_BASE = 0x1000
RAM_BASE = 0x20000000
RAM_SIZE = 0x10000
R9_INIT = RAM_BASE + 0x4000
SP_INIT = RAM_BASE + 0xC000

LOW = [f"r{i}" for i in range(8)]
HIGH_OK = ["r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r10", "r11", "r12", "lr"]
SRC = HIGH_OK
CONDS = ["eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc", "hi", "ls", "ge", "lt", "gt", "le"]
SHIFTS = ["lsl", "lsr", "asr", "ror"]


def lo():
    return random.choice(LOW)


def hi():
    return random.choice(HIGH_OK)


def modimm():
    b = random.randrange(256)
    k = random.randrange(5)
    if k == 0:
        return b
    if k == 1:
        return b | (b << 16)
    if k == 2:
        return (b << 8) | (b << 24)
    if k == 3:
        return b * 0x01010101
    rot = random.randrange(8, 32)
    v = 0x80 | (b & 0x7F)
    return ((v >> rot) | (v << (32 - rot))) & 0xFFFFFFFF


def shift_suffix():
    k = random.randrange(6)
    if k == 0:
        return ""
    if k == 1:
        return ", rrx"
    s = random.choice(SHIFTS)
    n = random.randrange(1, 32) if s in ("lsl", "ror") else random.randrange(1, 33)
    return f", {s} #{n}"


def t16_dp():
    op = random.choice(["ands", "eors", "orrs", "bics", "mvns", "lsls", "lsrs", "asrs", "rors",
                        "adcs", "sbcs", "tst", "cmp", "cmn", "muls"])
    a, b = lo(), lo()
    if op == "muls":
        return [f"muls {a}, {b}, {a}"]
    if op in ("tst", "cmp", "cmn", "mvns"):
        return [f"{op} {a}, {b}"]
    return [f"{op} {a}, {a}, {b}"]


def t16_misc():
    k = random.randrange(12)
    a, b, c = lo(), lo(), lo()
    return [[f"adds {a}, {b}, {c}"], [f"subs {a}, {b}, {c}"], [f"adds {a}, {b}, #{random.randrange(8)}"],
            [f"subs {a}, {b}, #{random.randrange(8)}"], [f"movs {a}, #{random.randrange(256)}"],
            [f"cmp {a}, #{random.randrange(256)}"], [f"adds {a}, #{random.randrange(256)}"],
            [f"subs {a}, #{random.randrange(256)}"], [f"lsls {a}, {b}, #{random.randrange(32)}"],
            [f"lsrs {a}, {b}, #{random.randrange(1, 33)}"], [f"asrs {a}, {b}, #{random.randrange(1, 33)}"],
            [f"rsbs {a}, {b}, #0"]][k]


def t16_ext():
    op = random.choice(["sxth", "sxtb", "uxth", "uxtb", "rev", "rev16", "revsh"])
    return [f"{op} {lo()}, {lo()}"]


def t32_dp_reg():
    op = random.choice(["add", "adds", "sub", "subs", "rsb", "rsbs", "adc", "adcs", "sbc", "sbcs",
                        "and", "ands", "orr", "orrs", "orn", "orns", "eor", "eors", "bic", "bics"])
    return [f"{op}.w {hi()}, {hi()}, {hi()}{shift_suffix()}"]


def t32_dp_cmp():
    op = random.choice(["tst", "teq", "cmp", "cmn"])
    return [f"{op}.w {hi()}, {hi()}{shift_suffix()}"]


def t32_mov():
    op = random.choice(["mov", "movs", "mvn", "mvns"])
    sh = shift_suffix()
    if op.startswith("mov") and sh:
        s = sh.split(",")[1].strip().split()
        if s[0] == "rrx":
            return [f"{'rrxs' if op == 'movs' else 'rrx'} {hi()}, {hi()}"]
        name = s[0] + ("s" if op == "movs" else "")
        return [f"{name}.w {hi()}, {hi()}, {s[1]}"]
    return [f"{op}.w {hi()}, {hi()}{sh}"]


def t32_dp_imm():
    op = random.choice(["add", "adds", "sub", "subs", "rsb", "rsbs", "adc", "adcs", "sbc", "sbcs",
                        "and", "ands", "orr", "orrs", "orn", "orns", "eor", "eors", "bic", "bics"])
    return [f"{op}.w {hi()}, {hi()}, #{modimm()}"]


def t32_imm_misc():
    k = random.randrange(9)
    d, n = hi(), hi()
    if k == 0:
        return [f"mov.w {d}, #{modimm()}"]
    if k == 1:
        return [f"mvns.w {d}, #{modimm()}"]
    if k == 2:
        return [f"{random.choice(['tst', 'teq', 'cmp', 'cmn'])}.w {n}, #{modimm()}"]
    if k == 3:
        return [f"addw {d}, {n}, #{random.randrange(4096)}"]
    if k == 4:
        return [f"subw {d}, {n}, #{random.randrange(4096)}"]
    if k == 5:
        return [f"movw {d}, #{random.randrange(65536)}"]
    if k == 6:
        return [f"movt {d}, #{random.randrange(65536)}"]
    if k == 7:
        return [f"movs.w {d}, #{modimm()}"]
    return [f"ands.w {d}, {n}, #{modimm()}"]


def t32_bits():
    k = random.randrange(8)
    d, n = hi(), hi()
    lsb = random.randrange(32)
    w = random.randrange(1, 33 - lsb)
    if k == 0:
        return [f"ubfx {d}, {n}, #{lsb}, #{w}"]
    if k == 1:
        return [f"sbfx {d}, {n}, #{lsb}, #{w}"]
    if k == 2:
        return [f"bfi {d}, {n}, #{lsb}, #{w}"]
    if k == 3:
        return [f"bfc {d}, #{lsb}, #{w}"]
    if k == 4:
        sh = random.choice(["", f", lsl #{random.randrange(32)}", f", asr #{random.randrange(1, 32)}"])
        return [f"ssat {d}, #{random.randrange(1, 33)}, {n}{sh}"]
    if k == 5:
        sh = random.choice(["", f", lsl #{random.randrange(32)}", f", asr #{random.randrange(1, 32)}"])
        return [f"usat {d}, #{random.randrange(0, 32)}, {n}{sh}"]
    if k == 6:
        return [f"{random.choice(['rbit', 'clz', 'rev.w', 'rev16.w', 'revsh.w'])} {d}, {n}"]
    return [f"{random.choice(['sxth', 'sxtb', 'uxth', 'uxtb'])}.w {d}, {n}, ror #{random.choice([0, 8, 16, 24])}"]


def t32_shift_reg():
    op = random.choice(["lsl", "lsls", "lsr", "lsrs", "asr", "asrs", "ror", "rors"])
    return [f"{op}.w {hi()}, {hi()}, {hi()}"]


def t32_mul():
    k = random.randrange(9)
    a, b, c, d = hi(), hi(), hi(), hi()
    while d == a:
        d = hi()
    if k == 0:
        return [f"mul.w {a}, {b}, {c}"]
    if k == 1:
        return [f"mla {a}, {b}, {c}, {d}"]
    if k == 2:
        return [f"mls {a}, {b}, {c}, {d}"]
    if k == 3:
        return [f"umull {a}, {d}, {b}, {c}"]
    if k == 4:
        return [f"smull {a}, {d}, {b}, {c}"]
    if k == 5:
        return [f"umlal {a}, {d}, {b}, {c}"]
    if k == 6:
        return [f"smlal {a}, {d}, {b}, {c}"]
    if k == 7:
        return [f"udiv {a}, {b}, {c}"]
    return [f"sdiv {a}, {b}, {c}"]


def t_it():
    cond = random.choice(CONDS)
    n = random.randrange(1, 5)
    pattern = "".join(random.choice("te") for _ in range(n - 1))
    inv = {"eq": "ne", "ne": "eq", "cs": "cc", "cc": "cs", "mi": "pl", "pl": "mi", "vs": "vc", "vc": "vs",
           "hi": "ls", "ls": "hi", "ge": "lt", "lt": "ge", "gt": "le", "le": "gt"}
    lines = [f"it{pattern} {cond}"]
    conds = [cond] + [cond if p == "t" else inv[cond] for p in pattern]
    for c in conds:
        k = random.randrange(5)
        if k == 0:
            lines.append(f"add{c} {lo()}, {lo()}, {lo()}")
        elif k == 1:
            lines.append(f"mov{c} {lo()}, #{random.randrange(256)}")
        elif k == 2:
            lines.append(f"adds{c}.w {hi()}, {hi()}, #{modimm()}")
        elif k == 3:
            lines.append(f"lsl{c} {lo()}, {lo()}, #{random.randrange(1, 32)}")
        else:
            lines.append(f"cmp{c} {lo()}, {lo()}")
    return lines


def t_mem():
    k = random.randrange(22)
    d, d2 = hi(), hi()
    while d2 == d:
        d2 = hi()
    off = random.randrange(0, 256)
    if k == 0:
        return [f"ldr.w {d}, [r9, #{off}]"]
    if k == 1:
        return [f"str.w {d}, [r9, #{off}]"]
    if k == 2:
        return [f"ldrb.w {d}, [r9, #{off}]"]
    if k == 3:
        return [f"ldrsb.w {d}, [r9, #-{off}]"]
    if k == 4:
        return [f"ldrh.w {d}, [r9, #{off}]"]
    if k == 5:
        return [f"ldrsh.w {d}, [r9, #{off}]"]
    if k == 6:
        return [f"strb.w {d}, [r9, #-{off}]"]
    if k == 7:
        return [f"strh.w {d}, [r9, #{off}]"]
    if k == 8:
        return [f"ldr {d}, [r9, #{4 * random.randrange(0, 63)}]!"]
    if k == 9:
        return [f"str {d}, [r9], #-{4 * random.randrange(0, 63)}"]
    if k == 10:
        return [f"movs r7, #{random.randrange(256)}", f"ldr.w {d if d != 'r7' else 'r0'}, [r9, r7, lsl #{random.randrange(4)}]"]
    if k == 11:
        return [f"movs r7, #{random.randrange(256)}", f"strh.w {d}, [r9, r7]"]
    if k == 12:
        a, b = random.sample(["r0", "r1", "r2", "r3", "r4", "r5"], 2)
        return ["mov r6, r9", f"ldr {a}, [r6, #{4 * random.randrange(32)}]", f"strb {b}, [r6, #{random.randrange(32)}]",
                f"ldrh {a}, [r6, #{2 * random.randrange(32)}]", f"str {b}, [r6, #{4 * random.randrange(32)}]"]
    if k == 13:
        regs = sorted(random.sample(range(0, 9), random.randrange(1, 6)))
        regs2 = sorted(random.sample(range(0, 9), len(regs)))
        return [f"push {{{', '.join('r%d' % r for r in regs)}}}", f"pop {{{', '.join('r%d' % r for r in regs2)}}}"]
    if k == 14:
        regs = sorted(random.sample([0, 1, 2, 3, 4, 5, 8, 10, 11, 12], random.randrange(1, 6)))
        return [f"stmia r9!, {{{', '.join('r%d' % r for r in regs)}}}", f"ldmdb r9, {{{', '.join('r%d' % r for r in regs)}}}"]
    if k == 15:
        regs = sorted(random.sample([0, 1, 2, 3, 4, 5, 8, 10, 11, 12], random.randrange(1, 6)))
        return [f"stmdb r9, {{{', '.join('r%d' % r for r in regs)}}}", f"ldmia.w r9, {{{', '.join('r%d' % r for r in regs)}}}"]
    if k == 16:
        return [f"strd {d}, {d2}, [r9, #{4 * random.randrange(0, 60)}]"]
    if k == 17:
        return [f"ldrd {d}, {d2}, [r9, #-{4 * random.randrange(0, 60)}]"]
    if k == 18:
        return [f"ldrd {d}, {d2}, [r9], #{4 * random.randrange(0, 60)}"]
    if k == 19:
        return [f"ldrex {d}, [r9, #{4 * random.randrange(0, 60)}]", f"strex {d2}, {d}, [r9]"]
    if k == 20:
        return [f"ldr {d}, =0x{random.getrandbits(32):08x}"]
    a, b = random.sample(["r0", "r1", "r2", "r3", "r4", "r5"], 2)
    return ["mov r6, r9", f"str {a}, [r6, #{4 * random.randrange(32)}]", f"ldr.w {b}, [r6, #{random.randrange(0, 64)}]"]


def t_branch():
    k = random.randrange(7)
    if k == 0:
        return [f"cmp {lo()}, {lo()}", f"b{random.choice(CONDS)} 1f", f"adds {lo()}, #1", "1:"]
    if k == 1:
        return [f"cmp.w {hi()}, {hi()}", f"b{random.choice(CONDS)}.w 1f", f"adds {lo()}, #1", "nop.w", "1:"]
    if k == 2:
        return [f"{random.choice(['cbz', 'cbnz'])} {lo()}, 1f", f"adds {lo()}, #1", "1:"]
    if k == 3:
        return ["bl 1f", f"adds {lo()}, #1", "1:", "mov r1, lr"]
    if k == 4:
        return ["adr r7, 1f + 1", "bx r7", "nop", ".align 2", "1:", f"movs {lo()}, #5"]
    if k == 5:
        idx = random.randrange(3)
        return [f"movs r7, #{idx}", "tbb [pc, r7]", "2:", ".byte (3f-2b)/2, (4f-2b)/2, (5f-2b)/2", ".align 1",
                "3:", "movs r1, #1", "b 6f", "4:", "movs r1, #2", "b 6f", "5:", "movs r1, #3", "6:"]
    return [f"msr APSR_nzcvq, {hi()}", f"mrs {hi()}, APSR"]


GENERATORS = [t16_dp, t16_misc, t16_ext, t32_dp_reg, t32_dp_cmp, t32_mov, t32_dp_imm, t32_imm_misc,
              t32_bits, t32_shift_reg, t32_mul, t_it, t_mem, t_branch]


def rand_val():
    k = random.randrange(10)
    if k == 0:
        return random.choice([0, 1, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0xFFFF, 0x8000, 0xFF, 0x80])
    if k < 4:
        return random.randrange(256)
    return random.getrandbits(32)


def build(blocks, tool):
    with tempfile.TemporaryDirectory() as td:
        src = ["\t.syntax unified", "\t.thumb", "\t.cpu cortex-m3", "\t.text", "\t.global _start", "_start:"]
        for i, b in enumerate(blocks):
            src.append(f"\t.global bs{i}\nbs{i}:")
            for line in b:
                src.append(line if line.endswith(":") or line.startswith(".") else "\t" + line)
            src.append(f"\t.global be{i}\nbe{i}:\n\tnop\n\tnop\n\t.ltorg\n\t.align 2")
        s = os.path.join(td, "t.s")
        o = os.path.join(td, "t.o")
        e = os.path.join(td, "t.elf")
        with open(s, "w") as f:
            f.write("\n".join(src) + "\n")
        r = subprocess.run([os.path.join(tool, "arm-none-eabi-as"), "-mimplicit-it=never", "-o", o, s],
                           capture_output=True, text=True)
        if r.returncode:
            return None, r.stderr
        subprocess.run([os.path.join(tool, "arm-none-eabi-ld"), f"-Ttext=0x{CODE_BASE:x}", "-e", "_start", "-o", e, o],
                       check=True, capture_output=True)
        with open(e, "rb") as f:
            elf = ELFFile(f)
            text = elf.get_section_by_name(".text")
            data = text.data()
            syms = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
        return (bytes(data), syms), None


def main():
    lib_path, tool = sys.argv[1], sys.argv[2]
    count = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    random.seed(seed)
    lib = ctypes.CDLL(lib_path)
    lib.t_new.restype = ctypes.c_void_p
    lib.t_new.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.t_set.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
    lib.t_run_until.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int]
    lib.t_get.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32)]
    lib.t_lockup.argtypes = [ctypes.c_void_p]
    lib.t_free.argtypes = [ctypes.c_void_p]

    blocks = []
    while len(blocks) < count:
        n = random.randrange(1, 5)
        blk = []
        for _ in range(n):
            blk += random.choice(GENERATORS)()
        blocks.append(blk)
    # Ensambla en lotes; descarta bloques que el ensamblador rechaza (combinaciones inválidas).
    good = []
    batch = 200
    for i in range(0, len(blocks), batch):
        chunk = blocks[i:i + batch]
        res, err = build(chunk, tool)
        if res is None:
            for b in chunk:
                r1, e1 = build([b], tool)
                if r1 is not None:
                    good.append((b, r1))
        else:
            good.append((chunk, res))

    code_size = 0x40000
    reg_ids = [getattr(A, f"UC_ARM_REG_R{i}") for i in range(13)] + [A.UC_ARM_REG_SP, A.UC_ARM_REG_LR, A.UC_ARM_REG_PC]
    fails = 0
    tested = 0
    for entry in good:
        chunk, (data, syms) = entry
        if isinstance(chunk[0], str):
            chunk = [chunk]
        code = (ctypes.c_uint8 * code_size)()
        ctypes.memmove(ctypes.addressof(code) + CODE_BASE, data, len(data))
        for bi, blk in enumerate(chunk):
            start, end = syms[f"bs{bi}"], syms[f"be{bi}"]
            regs = [rand_val() for _ in range(16)]
            regs[9] = R9_INIT
            regs[13] = SP_INIT
            regs[15] = start
            apsr = random.getrandbits(5) << 27
            ram_init = bytes(random.getrandbits(8) for _ in range(256)) * (RAM_SIZE // 256)

            ram = (ctypes.c_uint8 * RAM_SIZE).from_buffer_copy(ram_init)
            h = lib.t_new(code, code_size, ram, RAM_SIZE)
            arr = (ctypes.c_uint32 * 16)(*regs)
            lib.t_set(h, arr, apsr)
            steps = lib.t_run_until(h, end, 200)
            out = (ctypes.c_uint32 * 16)()
            ap = ctypes.c_uint32()
            lib.t_get(h, out, ctypes.byref(ap))
            lock = lib.t_lockup(h)
            mine_regs = list(out)
            mine_apsr = ap.value
            mine_ram = bytes(ram)
            lib.t_free(h)

            uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
            try:
                uc.ctl_set_cpu_model(A.UC_CPU_ARM_CORTEX_M3)
            except Exception:
                pass
            uc.mem_map(0, code_size)
            uc.mem_map(RAM_BASE, RAM_SIZE)
            uc.mem_write(0, bytes(code))
            uc.mem_write(RAM_BASE, ram_init)
            for i in range(15):
                uc.reg_write(reg_ids[i], regs[i])
            uc.reg_write(A.UC_ARM_REG_XPSR, apsr | 0x01000000)
            uerr = None
            try:
                uc.emu_start(start | 1, end, count=500)
            except UcError as ex:
                uerr = str(ex)
            u_regs = [uc.reg_read(rid) for rid in reg_ids[:15]] + [uc.reg_read(A.UC_ARM_REG_PC)]
            u_apsr = uc.reg_read(A.UC_ARM_REG_XPSR) & 0xF8000000
            u_ram = bytes(uc.mem_read(RAM_BASE, RAM_SIZE))
            tested += 1
            bad = []
            if uerr:
                bad.append(f"unicorn error {uerr}")
            if lock or steps >= 200:
                bad.append("lockup/no termina en el núcleo propio")
            for i in range(16):
                if mine_regs[i] != u_regs[i]:
                    bad.append(f"r{i}: mio={mine_regs[i]:08x} unicorn={u_regs[i]:08x}")
            if mine_apsr != u_apsr:
                bad.append(f"apsr: mio={mine_apsr:08x} unicorn={u_apsr:08x}")
            if mine_ram != u_ram:
                diffs = [i for i in range(RAM_SIZE) if mine_ram[i] != u_ram[i]]
                bad.append(f"ram difiere en {len(diffs)} bytes, primero en +0x{diffs[0]:x}")
            if bad:
                fails += 1
                if fails <= 25:
                    print("FALLA:", " | ".join(blk))
                    print("   init:", " ".join(f"{v:08x}" for v in regs), f"apsr={apsr:08x}")
                    for b in bad:
                        print("   ", b)
    print(f"bloques probados: {tested}, fallas: {fails}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()

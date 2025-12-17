import random
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class CoreState:
    # Hot addresses: reused a lot (temporal locality)
    hot_addrs: List[int]
    # Loop region: repeated loop-like accesses
    loop_base: int
    loop_len: int
    loop_stride: int
    loop_i: int = 0
    # Random-walk state: spatial locality with occasional jumps
    walk_addr: int = 0

def weighted_choice(rng: random.Random, items: List[Tuple[str, float]]) -> str:
    total = sum(w for _, w in items)
    x = rng.random() * total
    acc = 0.0
    for v, w in items:
        acc += w
        if x <= acc:
            return v
    return items[-1][0]

def gen_hot_set(rng: random.Random, base: int, span: int, count: int, align: int = 64) -> List[int]:
    s = set()
    while len(s) < count:
        a = base + rng.randrange(0, span)
        a = (a // align) * align
        s.add(a)
    return list(s)

def clamp_align(addr: int, low: int, high: int, align: int = 64) -> int:
    addr = max(low, min(high - 1, addr))
    return (addr // align) * align

def main(
    seed: int = 1234,
    num_cores: int = 16,
    t_max: int = 700000,
    target_accesses_per_core: int = 100000,
    out_prefix: str = "core",
):
    rng = random.Random(seed)

    # Address layout (you can tweak these)
    # Shared region: small-ish so collisions happen often (good for coherence)
    SHARED_BASE = 0x1000_0000
    SHARED_SIZE = 0x0000_8000  # 32 KB
    # Private regions: disjoint per core
    PRIV_BASE0 = 0x0000_0000
    PRIV_SIZE = 0x0001_0000    # 64 KB per core

    # Shared hot set: many accesses will reuse these addresses
    shared_hot = gen_hot_set(rng, SHARED_BASE, SHARED_SIZE, count=64, align=64)

    # Per-core states
    cores: List[CoreState] = []
    for c in range(num_cores):
        priv_base = PRIV_BASE0 + c * 0x0010_0000  # separate big gaps per core
        hot = gen_hot_set(rng, priv_base, PRIV_SIZE, count=64, align=64)

        loop_base = priv_base + rng.randrange(0, PRIV_SIZE // 2)
        loop_base = (loop_base // 64) * 64
        loop_len = rng.choice([16, 32, 64, 128])  # number of steps in loop
        loop_stride = rng.choice([64, 128, 256])  # bytes per step

        st = CoreState(
            hot_addrs=hot,
            loop_base=loop_base,
            loop_len=loop_len,
            loop_stride=loop_stride,
            loop_i=0,
            walk_addr=hot[rng.randrange(len(hot))],
        )
        cores.append(st)

    # Per-core output
    files = [open(f"{out_prefix}{c}.trace", "w") for c in range(num_cores)]
    access_counts = [0] * num_cores

    # Probability that a core does an access at a timestamp.
    # We want ~10k accesses across 70k timestamps => ~0.142 on average.
    # Give slight per-core variation to avoid being too uniform.
    base_p = target_accesses_per_core / t_max  # ~0.142857
    p_core = [max(0.05, min(0.35, base_p * rng.uniform(0.9, 1.1))) for _ in range(num_cores)]

    # Helper to choose R/W with a slight bias (reads more common)
    def choose_rw() -> str:
        return "R" if rng.random() < 0.7 else "W"

    # Address selection model:
    # - shared is more likely than private overall (as you asked)
    # - within each space, prefer hot set, then loop, then walk, then random
    REGION_SHARED_PROB = 0.62  # shared access chance higher than private

    for ts in range(t_max + 1):
        # For each core, maybe emit one access at this timestamp
        for c in range(num_cores):
            if access_counts[c] >= target_accesses_per_core:
                continue

            if rng.random() >= p_core[c]:
                continue  # no access at this timestamp

            is_shared = (rng.random() < REGION_SHARED_PROB)
            rw = choose_rw()

            st = cores[c]
            if is_shared:
                mode = weighted_choice(rng, [
                    ("hot",   0.55),
                    ("loop",  0.18),
                    ("walk",  0.22),
                    ("rand",  0.05),
                ])
                low, high = SHARED_BASE, SHARED_BASE + SHARED_SIZE
                hot_set = shared_hot
            else:
                priv_base = PRIV_BASE0 + c * 0x0010_0000
                low, high = priv_base, priv_base + PRIV_SIZE
                mode = weighted_choice(rng, [
                    ("hot",   0.48),
                    ("loop",  0.25),
                    ("walk",  0.22),
                    ("rand",  0.05),
                ])
                hot_set = st.hot_addrs

            if mode == "hot":
                # Mostly reuse a small set, but with a little churn
                if rng.random() < 0.92:
                    addr = hot_set[rng.randrange(len(hot_set))]
                else:
                    addr = clamp_align(low + rng.randrange(0, high - low), low, high, 64)

            elif mode == "loop":
                # A repeated stride loop pattern
                addr = st.loop_base + (st.loop_i % st.loop_len) * st.loop_stride
                addr = clamp_align(addr, low, high, 64)
                st.loop_i += 1
                # Occasionally reset loop to simulate loop restarting
                if rng.random() < 0.02:
                    st.loop_i = 0

            elif mode == "walk":
                # Random walk with strong spatial locality + occasional jump
                if rng.random() < 0.12:
                    # jump
                    addr = clamp_align(low + rng.randrange(0, high - low), low, high, 64)
                    st.walk_addr = addr
                else:
                    step = rng.choice([-256, -128, -64, 64, 128, 256])
                    st.walk_addr = clamp_align(st.walk_addr + step, low, high, 64)
                addr = st.walk_addr

            else:  # "rand"
                addr = clamp_align(low + rng.randrange(0, high - low), low, high, 64)

            files[c].write(f"{ts} {rw} 0x{addr:x}\n")
            access_counts[c] += 1

    for f in files:
        f.close()

    # Simple report
    for c in range(num_cores):
        print(f"core{c}: {access_counts[c]} accesses, file={out_prefix}{c}.trace")
    print(f"timestamp range: 0..{t_max}")

if __name__ == "__main__":
    main()

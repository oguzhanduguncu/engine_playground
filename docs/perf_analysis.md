# Pack installation

sudo apt install linux-perf
sudo apt install valgrind
sudo apt install kcachegrind

# Build

cmake -B build-bench -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build-bench


# Perf analysis

-- hardware counters summary:
- perf stat ./build-bench/bench_sim

-- flame graph / hotspot analysis:
- perf record -g ./build-bench/bench_sim
- perf report

# Callgrind output

valgrind --tool=callgrind --callgrind-out-file=cg.out ./build-bench/bench_sim

# Visualization

callgrind_annotate --tree=both cg.out | less
callgrind_annotate cg.out | grep -A 50 "Profile data file"

kcachegrind cg.out

lscpu | grep -i cache

# Get actual cache params

lscpu | grep -i cache

───────┬──────────────────────┬───────────────┬───────────┐                                                                                                                                                                                                                                                      
│ Level │         Size         │ Associativity │ Line Size │                                                                                                                                                                                                                                                      
├───────┼──────────────────────┼───────────────┼───────────┤                                                                                                                                                                                                                                                      
│ L1i   │ 128 KiB / 4 = 32 KiB │ ?             │ 64 B      │                                                                                                                                                                                                                                                      
├───────┼──────────────────────┼───────────────┼───────────┤                                                                                                                                                                                                                                                      
│ L1d   │ 192 KiB / 4 = 48 KiB │ 12-way        │ 64 B      │                                                                                                                                                                                                                                                      
├───────┼──────────────────────┼───────────────┼───────────┤                                                                                                                                                                                                                                                      
│ L2    │ 5 MiB / 4 = 1.25 MiB │ 20-way        │ 64 B      │
├───────┼──────────────────────┼───────────────┼───────────┤
│ L3    │ 8 MiB (shared)       │ ?             │ 64 B      │
└───────┴──────────────────────┴───────────────┴───────────┘

Flag Format

- --I1=< size>,< assoc>,<linesize>,<assoc>,<line_size>   # L1 instruction cache
- --D1=< size>,< assoc>,<line_size>   # L1 data cache
- --LL=< size>,< assoc>,<line_size>   # Last-level cache (L3)

# Cache performance analysis

Notes:
- Valgrind only models 3 levels (I1, D1, LL) — L2 is not directly configurable, LL maps to L3
- L1i associativity and L3 associativity aren't exposed by getconf on this system — 8 and 16 are safe typical defaults for Intel/AMD
- The 4 "instances" in lscpu are per-core caches, so divide total by 4 for per-core size

valgrind --tool=callgrind --cache-sim=yes --branch-sim=yes --I1=32768,8,64 --D1=49152,12,64 --LL=8388608,16,64 --callgrind-out-file=cg_cache.out ./build-bench/bench_sim

(warning: L3 cache found, using its data for the LL simulation.) It finds the last cache of the system when valgrind command runs.

# Visualization

kcachegrind cg_cache.out

- D1mr — L1 data read misses (cache thrashing)
- DLmr — LL (L3) read misses (actual RAM hits, most expensive)
- DLmw — LL write misses
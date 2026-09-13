# Build Guide: Assembly Optimizations

This guide explains how to build and troubleshoot the assembly optimizations for the flight control simulation.

## Quick Start

### Linux/macOS

```bash
# Clone the repository
git clone https://github.com/ambujmalik/flight-control-simulation.git
cd flight-control-simulation

# Install Python dependencies
pip install matplotlib ipywidgets numpy scipy numba pytest

# Build assembly optimizations (optional)
bash build_asm.sh

# Run tests
pytest test_assembly_optimizations.py -v

# Run the simulation
jupyter notebook
# Then run simulation.py in a Jupyter cell
```

## Detailed Build Steps

### 1. Install Dependencies

#### Python Packages

```bash
pip install --upgrade pip setuptools wheel

# Core dependencies
pip install numpy scipy matplotlib ipywidgets numba

# Development (for building extensions)
pip install scikit-build cmake

# Testing
pip install pytest pytest-cov
```

#### System Dependencies

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install -y \
    python3-dev \
    build-essential \
    nasm \
    gcc \
    gdb
```

**macOS:**
```bash
# Install Xcode command line tools
xcode-select --install

# Install NASM via Homebrew
brew install nasm
```

**CentOS/RHEL:**
```bash
sudo yum groupinstall "Development Tools"
sudo yum install nasm python3-devel
```

### 2. Build Assembly Optimizations

```bash
bash build_asm.sh
```

**What this does:**
1. Checks for NASM assembler
2. Assembles `flight_control_asm.asm` → `flight_control_asm.o`
3. Runs `python setup.py build_ext --inplace`
4. Produces `flight_control_asm.*.so` (shared library)

**Expected output:**
```
Building assembly optimizations...
NASM version 2.15.05
flight_control_asm.asm:1: warning: label alone on line 'section .data'
(some warnings are normal)
running build_ext
building 'flight_control_asm' extension
...
Build complete!
```

### 3. Verify Installation

```bash
# Check if assembly module loads
python -c "import flight_control_asm; print('Assembly loaded successfully')"

# Check if Numba fallback works
python -c "from flight_control_optimized import OptimizedFlightControlSimulation; \
    s = OptimizedFlightControlSimulation(); \
    print(f'ASM available: {s.use_asm}')"

# Run the benchmark
python flight_control_optimized.py
```

## Troubleshooting

### Issue: NASM not found

**Symptom:**
```
NASM not found. Installing...
```

**Solution:**
```bash
# Ubuntu/Debian
sudo apt-get install nasm

# macOS
brew install nasm

# Verify installation
nasm -version
```

---

### Issue: Assembly build fails with "Permission denied"

**Symptom:**
```
./build_asm.sh: Permission denied
```

**Solution:**
```bash
chmod +x build_asm.sh
bash build_asm.sh
```

---

### Issue: Python development headers not found

**Symptom:**
```
fatal error: Python.h: No such file or directory
```

**Solution:**
```bash
# Ubuntu/Debian
sudo apt-get install python3-dev

# macOS (Homebrew Python)
brew install python-dev

# Or use conda
conda install python-dev
```

---

### Issue: ModuleNotFoundError: No module named 'numpy.core._multiarray_umath'

**Symptom:**
```
ModuleNotFoundError: No module named 'numpy.core._multiarray_umath'
```

**Solution:**
Update NumPy:
```bash
pip install --upgrade numpy
```

---

### Issue: "ImportError: No module named flight_control_asm"

**Symptom:**
```
Traceback (most recent call last):
  File "...", line 3, in <module>
    import flight_control_asm as fca
ImportError: No module named 'flight_control_asm'
```

**Solution:**
The assembly optimizations are optional. The simulation falls back to Numba automatically:

```python
# This is expected and safe
from flight_control_optimized import OptimizedFlightControlSimulation
sim = OptimizedFlightControlSimulation()
print(f"ASM available: {sim.use_asm}")  # May print False

# Simulation will still run, just with Numba instead of ASM
```

If you want assembly support, rebuild:
```bash
bash build_asm.sh
```

---

### Issue: Segmentation fault during simulation

**Symptom:**
```
Segmentation fault (core dumped)
python: line 1: 12345 Segmentation fault (core dumped)
```

**Solution:**
This usually indicates array size mismatch. Verify:

1. **Parameter array has 26 elements:**
   ```python
   params = np.array([
       rho, CL0, CLa, CLq, MAC, CLde, S, CD0, K, CDde,
       CY_beta, CY_dr, Cm0, Cm_alpha, Cm_q, Cm_de,
       Cl_beta, Cl_p, Cl_r, Cl_da, Cl_dr,
       Cn_beta, Cn_p, Cn_r, Cn_da, Cn_dr, wing_span
   ])
   assert params.shape == (27,)  # Total 27 elements
   ```

2. **State array has 12 elements:**
   ```python
   state = np.array([V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d])
   assert state.shape == (12,)
   ```

3. **Controls array has 4 elements:**
   ```python
   controls = np.array([delta_e, delta_a, delta_r, delta_t])
   assert controls.shape == (4,)
   ```

---

### Issue: Assembly code compiles but produces wrong results

**Symptom:**
```
Forces/moments are incorrect or NaN
Assembly output differs from Python baseline
```

**Solution:**

1. **Run tests to identify mismatch:**
   ```bash
   pytest test_assembly_optimizations.py::TestAssemblyVsNumba -v
   ```

2. **Compare assembly vs Numba output:**
   ```python
   from flight_control_optimized import OptimizedFlightControlSimulation
   import numpy as np
   
   sim = OptimizedFlightControlSimulation()
   state = np.array([...])  # your test state
   controls = np.array([...])
   params = np.array([...])
   
   # Get both results
   result_numba = sim.compute_forces_moments_numba(state, controls, params)
   result_asm = sim.compute_forces_moments_optimized(state, controls, params)
   
   # Compare
   print("Numba:    ", result_numba)
   print("Assembly: ", result_asm)
   print("Difference:", np.abs(result_numba - result_asm))
   ```

3. **Debug with GDB (Linux only):**
   ```bash
   gdb python
   (gdb) run -m pdb simulation.py
   # Set breakpoint at assembly call
   ```

4. **Check parameter offsets in assembly:**
   - Verify `ARCHITECTURE.md` parameter offsets match `flight_control_asm.asm`
   - Use `objdump` to disassemble and check offsets:
     ```bash
     objdump -d flight_control_asm.o | grep -A200 compute_forces_moments_asm
     ```

---

### Issue: Build works but tests fail

**Symptom:**
```
FAILED test_assembly_optimizations.py::TestAssemblyVsNumba::test_assembly_vs_numba_forces_moments
AssertionError: Assembly and Numba differ
```

**Solution:**

1. **Check numerical precision:**
   - Assembly uses 64-bit (double) precision
   - Small differences (< 1e-5) are expected due to rounding
   - Test tolerance is set to decimal=5 (10^-5 relative error)

2. **Enable verbose logging:**
   ```python
   import logging
   logging.basicConfig(level=logging.DEBUG)
   
   from flight_control_optimized import OptimizedFlightControlSimulation
   sim = OptimizedFlightControlSimulation()
   ```

3. **Run single failing test with more info:**
   ```bash
   pytest test_assembly_optimizations.py::TestAssemblyVsNumba::test_assembly_vs_numba_forces_moments -vv -s
   ```

---

### Issue: Compiler warnings about register usage

**Symptom:**
```
warning: register name expected
warning: invalid operand size for instruction
```

**Solution:**
These are NASM warnings and usually harmless. Common causes:

1. **Typo in register name:** Check for `xmx` (should be `xmm`)
2. **Size mismatch:** Use `movsd` for 8-byte doubles, `movss` for 4-byte floats
3. **Operand addressing:** Verify RIP-relative addressing with `[rel symbol]`

To suppress specific warnings, update `build_asm.sh`:
```bash
nasm -f elf64 -W-orphan-labels -o flight_control_asm.o flight_control_asm.asm
```

---

## Clean Build

To completely rebuild from scratch:

```bash
# Remove built artifacts
rm -f flight_control_asm.o
rm -f flight_control_asm.*.so
rm -rf build/
rm -rf *.egg-info/

# Clean Python cache
find . -type d -name __pycache__ -exec rm -rf {} +
find . -name "*.pyc" -delete

# Rebuild
bash build_asm.sh
```

## Platform-Specific Build Instructions

### Windows (Not Supported)

Assembly optimizations require x86-64 NASM syntax (elf64 format), which is not compatible with Windows (COFF/PE format).

**Workaround:** Use Windows Subsystem for Linux (WSL):
```bash
# In WSL terminal
wsl
bash build_asm.sh
```

Or use pre-built wheel from CI/CD if available.

### Apple Silicon (M1/M2/M3)

Current assembly is x86-64 only and will not work on ARM64.

**Solution:** Use Rosetta 2 emulation or build for native ARM64 (requires rewriting assembly kernels).

For now, Numba fallback works on Apple Silicon:
```bash
# Don't run build_asm.sh
# Just use Python dependencies
pip install numpy scipy matplotlib ipywidgets numba
python simulation.py
```

## Verifying Build Correctness

### Run Test Suite

```bash
# All tests (assembly tests skipped if not available)
pytest test_assembly_optimizations.py -v

# Only Numba tests
pytest test_assembly_optimizations.py::TestAssemblyOptimizations -v

# Only assembly tests (fails gracefully if ASM not available)
pytest test_assembly_optimizations.py::TestAssemblyVsNumba -v

# With coverage report
pytest test_assembly_optimizations.py --cov=. --cov-report=html
```

### Run Simulation and Verify Output

```python
from flight_control_optimized import OptimizedFlightControlSimulation
import numpy as np

sim = OptimizedFlightControlSimulation()

# Create test state
state = np.array([250.0, 0.05, 0.0, 0.0, 0.01, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 10000.0])
controls = np.array([0.0, 0.0, 0.0, 0.5])
params = np.ones(27) * 1.0  # Dummy params

# Run computation
result = sim.compute_forces_moments_optimized(state, controls, params)

# Verify result
print(f"ASM available: {sim.use_asm}")
print(f"Forces/Moments: {result}")
assert len(result) == 6, "Should return 6 elements"
assert np.all(np.isfinite(result)), "All elements should be finite"
print("✓ Build verification passed!")
```

## Performance Benchmarking

### Simple Benchmark

```bash
python flight_control_optimized.py
```

Expected output:
```
INFO:__main__:Assembly optimizations available
INFO:__main__:Python: 0.1234s, Assembly: 0.0987s
INFO:__main__:Speedup: 1.25x
```

### Detailed Performance Analysis

```bash
# Timing each component
python -m cProfile -s cumtime flight_control_optimized.py

# Profiling with perf (Linux)
perf record -g python flight_control_optimized.py
perf report

# Profiling with line_profiler
pip install line_profiler
kernprof -l -v flight_control_optimized.py
```

## Continuous Integration

### GitHub Actions

The repository should include `.github/workflows/build.yml`:

```yaml
name: Build Assembly Optimizations

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-python@v2
        with:
          python-version: '3.9'
      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install nasm
          pip install numpy pytest
      - name: Build
        run: bash build_asm.sh
      - name: Test
        run: pytest test_assembly_optimizations.py -v
```

## Support & Debugging

### Useful Commands for Debugging

```bash
# Check Python version
python --version

# Check NumPy version and include path
python -c "import numpy; print(numpy.get_include())"

# List loaded modules
python -c "import flight_control_asm; print(flight_control_asm.__file__)"

# Disassemble object file
objdump -d flight_control_asm.o

# Check dynamic dependencies
ldd flight_control_asm.*.so

# Trace system calls (Linux)
strace -e trace=open,openat python -c "import flight_control_asm"
```

### Getting Help

If problems persist:

1. **Check `ARCHITECTURE.md`** for technical details
2. **Run `pytest` with `-vv` flag** for detailed output
3. **Enable logging:**
   ```python
   import logging
   logging.basicConfig(level=logging.DEBUG)
   ```
4. **Open GitHub issue** with:
   - Platform (OS, Python version, CPU architecture)
   - Output of `bash build_asm.sh`
   - Full error message and traceback
   - Result of `pytest test_assembly_optimizations.py -v`

## References

- **NASM Manual:** https://www.nasm.us/doc/
- **x86-64 ABI:** https://refspecs.linuxbase.org/elf/x86-64-abi-0.99.pdf
- **NumPy C API:** https://numpy.org/doc/stable/reference/c-api/
- **Python C API:** https://docs.python.org/3/c-api/

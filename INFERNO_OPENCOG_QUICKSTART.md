# Inferno-OpenCog Quick Start Guide
## Get Started with Cognitive Kernel Services in 5 Minutes

This guide gets you up and running with Inferno-OpenCog cognitive kernel services quickly.

## Prerequisites

```bash
# Install kernel headers
sudo apt-get update
sudo apt-get install -y linux-headers-$(uname -r) build-essential

# Verify installation
ls /lib/modules/$(uname -r)/build
```

## Quick Build & Load

### 1. Build All Modules

```bash
cd kernel/inferno-cog
make
```

**Expected output:**
```
make -C /lib/modules/.../build M=.../kernel/inferno-cog modules
  CC [M]  .../kernel/inferno-cog/atomspace.o
  CC [M]  .../kernel/inferno-cog/ecan.o
  CC [M]  .../kernel/inferno-cog/pln.o
  CC [M]  .../kernel/inferno-cog/moses.o
  LD [M]  .../kernel/inferno-cog/inferno_cog_atomspace.ko
  LD [M]  .../kernel/inferno-cog/inferno_cog_ecan.ko
  LD [M]  .../kernel/inferno-cog/inferno_cog_pln.ko
  LD [M]  .../kernel/inferno-cog/inferno_cog_moses.ko
```

### 2. Load AtomSpace Module

```bash
sudo insmod inferno_cog_atomspace.ko
```

**Verify:**
```bash
lsmod | grep inferno_cog
dmesg | tail
```

**Expected output:**
```
inferno_cog: Loading Inferno-OpenCog AtomSpace module
inferno_cog: Initializing AtomSpace kernel module
inferno_cog: AtomSpace initialized successfully
inferno_cog: Max atoms: 16777216, OEIS A000081 depth: 0
inferno_cog: AtomSpace module loaded successfully
```

### 3. Load Additional Modules

```bash
sudo insmod inferno_cog_ecan.ko
sudo insmod inferno_cog_pln.ko
sudo insmod inferno_cog_moses.ko
```

### 4. Verify All Modules Loaded

```bash
lsmod | grep inferno_cog
```

**Expected:**
```
inferno_cog_moses          16384  0
inferno_cog_pln            16384  0
inferno_cog_ecan           24576  0
inferno_cog_atomspace      24576  3 inferno_cog_moses,inferno_cog_pln,inferno_cog_ecan
```

## Quick Test

### Test AtomSpace Operations

Create a simple C test program:

```c
#include <linux/module.h>
#include <linux/kernel.h>
#include "../../include/dtesn/inferno_cog.h"

void test_atomspace(void)
{
    atom_id_t atom1, atom2;
    truth_value_t tv = {0.9, 0.8, 10};
    
    /* Create atoms */
    atom1 = atom_create(ATOM_TYPE_CONCEPT, "cat", tv);
    atom2 = atom_create(ATOM_TYPE_CONCEPT, "mammal", tv);
    
    printk("Created atoms: %llu, %llu\n", atom1, atom2);
    
    /* Clean up */
    atom_delete(atom1);
    atom_delete(atom2);
}
```

Or use kernel module test framework:

```bash
# Run built-in tests (if available)
sudo sh -c 'echo "test" > /proc/inferno_cog/atomspace_test'
dmesg | tail -20
```

## Basic Usage Examples

### Creating Atoms from Kernel Module

```c
#include "../../include/dtesn/inferno_cog.h"

/* Create a concept atom */
truth_value_t tv = {
    .strength = 0.9,      /* 90% confidence */
    .confidence = 0.8,    /* 80% evidence strength */
    .count = 10
};

atom_id_t cat = atom_create(ATOM_TYPE_CONCEPT, "cat", tv);

/* Update truth value */
tv.strength = 0.95;
atom_set_tv(cat, tv);

/* Retrieve truth value */
truth_value_t retrieved_tv;
atom_get_tv(cat, &retrieved_tv);

/* Delete atom */
atom_delete(cat);
```

### Using ECAN Attention

```c
#include "../../include/dtesn/inferno_cog.h"

/* Stimulate atom with attention */
ecan_stimulate(&global_ecan, cat_atom_id, +100);  /* Increase STI by 100 */

/* Spread importance from atom */
ecan_spread_importance(&global_ecan, cat_atom_id);

/* Update attentional focus */
ecan_update_af(&global_ecan);

/* Forget low-importance atoms */
int forgotten = ecan_forget(&global_ecan);
```

### PLN Inference (Stub)

```c
#include "../../include/dtesn/inferno_cog.h"

/* Add inference rule */
struct inference_rule rule = {
    .rule_id = 1,
    .name = "deduction",
    /* ... */
};
pln_add_rule(&global_pln, &rule);

/* Perform inference */
atom_id_t premises[] = {atom1, atom2};
atom_id_t conclusion;
pln_infer(&global_pln, premises, 2, &conclusion);
```

## Unloading Modules

```bash
sudo rmmod inferno_cog_moses
sudo rmmod inferno_cog_pln
sudo rmmod inferno_cog_ecan
sudo rmmod inferno_cog_atomspace
```

**Verify:**
```bash
lsmod | grep inferno_cog  # Should show nothing
dmesg | tail
```

**Expected:**
```
inferno_cog: Unloading MOSES module
inferno_cog: MOSES shutdown complete
inferno_cog: Unloading PLN module
inferno_cog: PLN shutdown complete
inferno_cog: Unloading ECAN module
inferno_cog: ECAN shutdown complete
inferno_cog: Unloading Inferno-OpenCog AtomSpace module
inferno_cog: Shutting down AtomSpace
inferno_cog: Freed 0 atoms
inferno_cog: AtomSpace shutdown complete
```

## Monitoring and Debugging

### View Kernel Logs

```bash
# Real-time monitoring
sudo dmesg -w | grep inferno_cog

# View recent logs
dmesg | grep inferno_cog | tail -50
```

### Check Module Parameters

```bash
# View ECAN parameters
cat /sys/module/inferno_cog_ecan/parameters/af_size
cat /sys/module/inferno_cog_ecan/parameters/af_rent

# View MOSES parameters
cat /sys/module/inferno_cog_moses/parameters/population_size
```

### Adjust Parameters at Runtime

```bash
# Change ECAN attentional focus size
echo 2000 | sudo tee /sys/module/inferno_cog_ecan/parameters/af_size

# Change MOSES population size
echo 200 | sudo tee /sys/module/inferno_cog_moses/parameters/population_size
```

## Performance Testing

### Simple Benchmark

```bash
# Time module loading
time sudo insmod inferno_cog_atomspace.ko

# Measure atom creation performance
# (Requires test module - see testing documentation)
```

## Troubleshooting

### Module Won't Load

**Issue:** `insmod: ERROR: could not insert module`

**Solutions:**
```bash
# Check detailed error
dmesg | tail

# Verify kernel version matches
uname -r
ls /lib/modules/$(uname -r)/build

# Rebuild modules
make clean && make
```

### Symbol Not Found Errors

**Issue:** `Unknown symbol in module`

**Solution:** Load modules in correct order:
1. inferno_cog_atomspace (base module)
2. inferno_cog_ecan (depends on atomspace)
3. inferno_cog_pln (depends on atomspace)
4. inferno_cog_moses (depends on atomspace)

### Out of Memory

**Issue:** Module fails to initialize

**Solution:**
```bash
# Check available memory
free -h

# Reduce module parameters
sudo insmod inferno_cog_atomspace.ko  # Default is safe
sudo insmod inferno_cog_ecan.ko af_size=500  # Reduce from 1000
```

## Next Steps

1. **Read Full Documentation**: See `INFERNO_OPENCOG_ARCHITECTURE.md`
2. **Explore Examples**: Check `examples/` directory
3. **Run Tests**: See `tests/` directory
4. **Develop Applications**: Use kernel APIs in your modules
5. **Contribute**: See `DEVELOPMENT.md`

## Architecture at a Glance

```
┌─────────────────────────────────────────────────────┐
│           Inferno-OpenCog Kernel Stack              │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌──────────┐  ┌──────┐  ┌─────┐  ┌───────┐      │
│  │AtomSpace │  │ ECAN │  │ PLN │  │ MOSES │      │
│  │  (Core)  │  │      │  │     │  │       │      │
│  └────┬─────┘  └───┬──┘  └──┬──┘  └───┬───┘      │
│       │            │        │         │           │
│       └────────────┴────────┴─────────┘           │
│                    │                               │
│         ┌──────────▼──────────┐                   │
│         │  DTESN Integration  │                   │
│         │  - OEIS A000081     │                   │
│         │  - P-System         │                   │
│         │  - ESN Reservoir    │                   │
│         └─────────────────────┘                   │
│                                                     │
└─────────────────────────────────────────────────────┘
           ▼              ▼              ▼
      Linux Kernel   Neuromorphic    9P Protocol
                     Hardware        (Future)
```

## Quick Reference Commands

```bash
# Build
make -C kernel/inferno-cog

# Load all
sudo insmod kernel/inferno-cog/inferno_cog_atomspace.ko
sudo insmod kernel/inferno-cog/inferno_cog_ecan.ko
sudo insmod kernel/inferno-cog/inferno_cog_pln.ko
sudo insmod kernel/inferno-cog/inferno_cog_moses.ko

# Monitor
dmesg -w | grep inferno_cog

# Unload all
sudo rmmod inferno_cog_moses inferno_cog_pln inferno_cog_ecan inferno_cog_atomspace

# Clean
make -C kernel/inferno-cog clean
```

## Support

- **Documentation**: `kernel/inferno-cog/README.md`
- **Architecture**: `INFERNO_OPENCOG_ARCHITECTURE.md`
- **Issues**: https://github.com/o9nn/echo.kern/issues
- **Logs**: `dmesg | grep inferno_cog`

---

**Welcome to Cognitive Kernel Programming! 🧠**

*Where thinking is not an application—it's part of the machine itself.*

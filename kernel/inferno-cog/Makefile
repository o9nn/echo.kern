# Makefile for Inferno-OpenCog Kernel Modules
# ============================================
# 
# Builds OpenCog cognitive primitives as native kernel modules
# integrated with Echo.Kern DTESN architecture.

obj-m += inferno_cog_atomspace.o
obj-m += inferno_cog_ecan.o
obj-m += inferno_cog_pln.o
obj-m += inferno_cog_moses.o

# Source files for each module
inferno_cog_atomspace-objs := atomspace.o
inferno_cog_ecan-objs := ecan.o
inferno_cog_pln-objs := pln.o
inferno_cog_moses-objs := moses.o

# Kernel build directory
KDIR ?= /lib/modules/$(shell uname -r)/build

# Current directory
PWD := $(shell pwd)

# Build flags
ccflags-y := -I$(PWD)/../../include -Wall -Wextra -O2

# Default target
all: modules

# Build kernel modules
modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean build artifacts
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f Module.symvers modules.order

# Install modules
install: modules
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# Help target
help:
	@echo "Inferno-OpenCog Kernel Module Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build all kernel modules (default)"
	@echo "  modules  - Build kernel modules"
	@echo "  clean    - Clean build artifacts"
	@echo "  install  - Install modules to system"
	@echo "  help     - Show this help message"
	@echo ""
	@echo "Modules:"
	@echo "  inferno_cog_atomspace - AtomSpace knowledge representation"
	@echo "  inferno_cog_ecan      - Economic Attention Networks"
	@echo "  inferno_cog_pln       - Probabilistic Logic Networks"
	@echo "  inferno_cog_moses     - Meta-Optimizing Semantic Evolutionary Search"
	@echo ""
	@echo "Note: Requires Linux kernel headers to be installed"

.PHONY: all modules clean install help

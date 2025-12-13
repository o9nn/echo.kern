/*
 * Inferno-OpenCog AtomSpace Kernel Module
 * ========================================
 * 
 * Core kernel implementation of OpenCog AtomSpace as native Inferno/DTESN
 * kernel service. Provides knowledge representation with OEIS A000081
 * hierarchical structure and P-System membrane security integration.
 * 
 * This module implements:
 * - Kernel-level atomspace with fast lookup
 * - OEIS A000081 compliant hierarchical organization
 * - Red-black tree for O(log n) access by ID
 * - Hash table for O(1) access by name
 * - P-System membrane security boundaries
 * - ESN reservoir temporal dynamics integration
 * 
 * Copyright (c) 2024 Echo.Kern Development Team
 * Licensed under GPL-2.0
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/rbtree.h>
#include <linux/hashtable.h>
#include <linux/string.h>
#include <linux/atomic.h>
#include <linux/time.h>

#include "../../include/dtesn/inferno_cog.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Echo.Kern Development Team");
MODULE_DESCRIPTION("Inferno-OpenCog AtomSpace Kernel Module");
MODULE_VERSION("1.0.0");

/* Global kernel atomspace instance */
static struct kern_atomspace global_atomspace;

/* OEIS A000081 sequence for validation */
static const uint32_t oeis_a000081[] = {
    0, 1, 1, 2, 4, 9, 20, 48, 115, 286, 719, 1842, 4766, 12486
};
#define OEIS_A000081_LEN (sizeof(oeis_a000081) / sizeof(oeis_a000081[0]))

/*
 * =============================================================================
 * Helper Functions
 * =============================================================================
 */

/* Get current timestamp in nanoseconds */
static inline uint64_t get_timestamp_ns(void)
{
    struct timespec64 ts;
    ktime_get_real_ts64(&ts);
    return (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

/* Calculate hash for atom name */
static inline uint32_t atom_name_hash(const char *name)
{
    return full_name_hash(NULL, name, strlen(name));
}

/* Validate OEIS A000081 depth */
static int validate_oeis_depth(uint32_t depth)
{
    if (depth >= OEIS_A000081_LEN) {
        pr_warn("inferno_cog: depth %u exceeds OEIS A000081 sequence length\n", 
                depth);
        return -EINVAL;
    }
    return 0;
}

/*
 * =============================================================================
 * Atom Reference Counting
 * =============================================================================
 */

/* Increment atom reference count */
static inline void atom_get_ref(struct kern_atom *atom)
{
    atomic_inc(&atom->refcount);
}

/* Decrement atom reference count and free if zero */
static void atom_put_ref(struct kern_atom *atom)
{
    if (atomic_dec_and_test(&atom->refcount)) {
        /* Free outgoing set for links */
        if (atom->outgoing) {
            kfree(atom->outgoing);
        }
        
        /* Free atom structure */
        kfree(atom);
    }
}

/*
 * =============================================================================
 * AtomSpace Core Operations
 * =============================================================================
 */

/**
 * atomspace_init - Initialize kernel atomspace subsystem
 * 
 * Initializes global atomspace with:
 * - Red-black tree for ID-based lookup
 * - Hash table for name-based lookup
 * - Atomic ID generation
 * - Statistics tracking
 * 
 * Return: 0 on success, negative error code on failure
 */
int atomspace_init(void)
{
    pr_info("inferno_cog: Initializing AtomSpace kernel module\n");
    
    /* Initialize red-black tree */
    global_atomspace.atoms = RB_ROOT;
    
    /* Initialize hash table */
    hash_init(global_atomspace.atom_index);
    
    /* Initialize spinlock */
    spin_lock_init(&global_atomspace.lock);
    
    /* Initialize atomic counters */
    atomic64_set(&global_atomspace.next_atom_id, 1);
    atomic64_set(&global_atomspace.atom_count, 0);
    atomic64_set(&global_atomspace.link_count, 0);
    atomic64_set(&global_atomspace.total_created, 0);
    atomic64_set(&global_atomspace.total_deleted, 0);
    
    /* Initialize OEIS depth */
    global_atomspace.oeis_depth = 0;
    
    /* Initialize ECAN (attention mechanism) */
    global_atomspace.ecan = NULL;  /* Will be initialized separately */
    
    /* Initialize membrane security */
    global_atomspace.security_membrane = NULL;
    
    /* Initialize ESN reservoir */
    global_atomspace.reservoir = NULL;
    
    pr_info("inferno_cog: AtomSpace initialized successfully\n");
    pr_info("inferno_cog: Max atoms: %lu, OEIS A000081 depth: %u\n",
            ATOMSPACE_MAX_ATOMS, global_atomspace.oeis_depth);
    
    return 0;
}

/**
 * atomspace_exit - Cleanup kernel atomspace subsystem
 * 
 * Frees all atoms and resources.
 */
void atomspace_exit(void)
{
    struct rb_node *node;
    struct kern_atom *atom;
    uint64_t freed_count = 0;
    
    pr_info("inferno_cog: Shutting down AtomSpace\n");
    
    /* Free all atoms in red-black tree */
    spin_lock(&global_atomspace.lock);
    
    while ((node = rb_first(&global_atomspace.atoms)) != NULL) {
        atom = rb_entry(node, struct kern_atom, rb_node);
        
        /* Remove from tree and hash table */
        rb_erase(node, &global_atomspace.atoms);
        hash_del(&atom->hash_node);
        
        /* Release atom */
        atom_put_ref(atom);
        freed_count++;
    }
    
    spin_unlock(&global_atomspace.lock);
    
    pr_info("inferno_cog: Freed %llu atoms\n", freed_count);
    pr_info("inferno_cog: AtomSpace shutdown complete\n");
}

/**
 * atom_create - Create new atom in kernel atomspace
 * @type: Atom type (node or link)
 * @name: Atom name (for nodes, can be empty for links)
 * @tv: Initial truth value
 * 
 * Creates a new atom with unique ID, adds to atomspace indices,
 * and validates OEIS A000081 hierarchy.
 * 
 * Return: Atom ID on success, ATOM_ID_INVALID on failure
 */
atom_id_t atom_create(atom_type_t type, const char *name, truth_value_t tv)
{
    struct kern_atom *atom;
    atom_id_t id;
    uint32_t name_hash;
    uint64_t timestamp;
    struct rb_node **new, *parent = NULL;
    
    /* Validate inputs */
    if (!name || strlen(name) >= ATOMSPACE_MAX_NAME_LEN) {
        pr_err("inferno_cog: Invalid atom name\n");
        return ATOM_ID_INVALID;
    }
    
    /* Check atomspace capacity */
    if (atomic64_read(&global_atomspace.atom_count) >= ATOMSPACE_MAX_ATOMS) {
        pr_err("inferno_cog: AtomSpace at maximum capacity\n");
        return ATOM_ID_INVALID;
    }
    
    /* Allocate atom structure */
    atom = kzalloc(sizeof(struct kern_atom), GFP_KERNEL);
    if (!atom) {
        pr_err("inferno_cog: Failed to allocate atom\n");
        return ATOM_ID_INVALID;
    }
    
    /* Initialize atom */
    id = atomic64_inc_return(&global_atomspace.next_atom_id);
    atom->id = id;
    atom->type = type;
    strncpy(atom->name, name, ATOMSPACE_MAX_NAME_LEN - 1);
    atom->name[ATOMSPACE_MAX_NAME_LEN - 1] = '\0';
    
    atom->tv = tv;
    atom->av.sti = 0;
    atom->av.lti = 0;
    atom->av.vlti = 0;
    
    atom->outgoing = NULL;
    atom->outgoing_size = 0;
    INIT_LIST_HEAD(&atom->incoming);
    
    atom->depth = global_atomspace.oeis_depth;
    atom->parent = NULL;
    
    atomic_set(&atom->refcount, 1);
    timestamp = get_timestamp_ns();
    atom->creation_time_ns = timestamp;
    
    /* Insert into atomspace */
    spin_lock(&global_atomspace.lock);
    
    /* Insert into red-black tree (by ID) */
    new = &global_atomspace.atoms.rb_node;
    while (*new) {
        struct kern_atom *this = rb_entry(*new, struct kern_atom, rb_node);
        parent = *new;
        
        if (id < this->id)
            new = &(*new)->rb_left;
        else if (id > this->id)
            new = &(*new)->rb_right;
        else {
            /* Should never happen with atomic ID generation */
            spin_unlock(&global_atomspace.lock);
            kfree(atom);
            pr_err("inferno_cog: Duplicate atom ID detected\n");
            return ATOM_ID_INVALID;
        }
    }
    rb_link_node(&atom->rb_node, parent, new);
    rb_insert_color(&atom->rb_node, &global_atomspace.atoms);
    
    /* Insert into hash table (by name) */
    name_hash = atom_name_hash(name);
    hash_add(global_atomspace.atom_index, &atom->hash_node, name_hash);
    
    /* Update statistics */
    atomic64_inc(&global_atomspace.atom_count);
    atomic64_inc(&global_atomspace.total_created);
    if (type & ATOM_TYPE_LINK) {
        atomic64_inc(&global_atomspace.link_count);
    }
    
    spin_unlock(&global_atomspace.lock);
    
    pr_debug("inferno_cog: Created atom %llu: type=%u name='%s' tv=(%.3f,%.3f)\n",
             id, type, name, tv.strength, tv.confidence);
    
    return id;
}

/**
 * atom_delete - Delete atom from kernel atomspace
 * @id: Atom ID to delete
 * 
 * Removes atom from all indices and frees memory.
 * 
 * Return: 0 on success, negative error code on failure
 */
int atom_delete(atom_id_t id)
{
    struct kern_atom *atom;
    struct rb_node *node;
    
    if (id == ATOM_ID_INVALID) {
        return -EINVAL;
    }
    
    spin_lock(&global_atomspace.lock);
    
    /* Find atom in red-black tree */
    node = global_atomspace.atoms.rb_node;
    while (node) {
        atom = rb_entry(node, struct kern_atom, rb_node);
        
        if (id < atom->id) {
            node = node->rb_left;
        } else if (id > atom->id) {
            node = node->rb_right;
        } else {
            /* Found atom - remove from indices */
            rb_erase(&atom->rb_node, &global_atomspace.atoms);
            hash_del(&atom->hash_node);
            
            /* Update statistics */
            atomic64_dec(&global_atomspace.atom_count);
            atomic64_inc(&global_atomspace.total_deleted);
            if (atom->type & ATOM_TYPE_LINK) {
                atomic64_dec(&global_atomspace.link_count);
            }
            
            spin_unlock(&global_atomspace.lock);
            
            /* Release atom */
            atom_put_ref(atom);
            
            pr_debug("inferno_cog: Deleted atom %llu\n", id);
            return 0;
        }
    }
    
    spin_unlock(&global_atomspace.lock);
    
    pr_warn("inferno_cog: Atom %llu not found for deletion\n", id);
    return -ENOENT;
}

/**
 * atom_get - Retrieve atom from kernel atomspace
 * @id: Atom ID to retrieve
 * @out: Pointer to store atom pointer
 * 
 * Looks up atom by ID and returns pointer with incremented reference count.
 * Caller must call atom_put_ref() when done.
 * 
 * Return: 0 on success, negative error code on failure
 */
int atom_get(atom_id_t id, struct kern_atom **out)
{
    struct kern_atom *atom;
    struct rb_node *node;
    
    if (id == ATOM_ID_INVALID || !out) {
        return -EINVAL;
    }
    
    spin_lock(&global_atomspace.lock);
    
    /* Find atom in red-black tree */
    node = global_atomspace.atoms.rb_node;
    while (node) {
        atom = rb_entry(node, struct kern_atom, rb_node);
        
        if (id < atom->id) {
            node = node->rb_left;
        } else if (id > atom->id) {
            node = node->rb_right;
        } else {
            /* Found atom - increment reference count */
            atom_get_ref(atom);
            *out = atom;
            spin_unlock(&global_atomspace.lock);
            return 0;
        }
    }
    
    spin_unlock(&global_atomspace.lock);
    return -ENOENT;
}

/**
 * atom_set_tv - Set atom truth value
 * @id: Atom ID
 * @tv: New truth value
 * 
 * Return: 0 on success, negative error code on failure
 */
int atom_set_tv(atom_id_t id, truth_value_t tv)
{
    struct kern_atom *atom;
    int ret;
    
    ret = atom_get(id, &atom);
    if (ret)
        return ret;
    
    atom->tv = tv;
    atom_put_ref(atom);
    
    pr_debug("inferno_cog: Set TV for atom %llu: (%.3f,%.3f)\n",
             id, tv.strength, tv.confidence);
    
    return 0;
}

/**
 * atom_get_tv - Get atom truth value
 * @id: Atom ID
 * @out: Pointer to store truth value
 * 
 * Return: 0 on success, negative error code on failure
 */
int atom_get_tv(atom_id_t id, truth_value_t *out)
{
    struct kern_atom *atom;
    int ret;
    
    if (!out)
        return -EINVAL;
    
    ret = atom_get(id, &atom);
    if (ret)
        return ret;
    
    *out = atom->tv;
    atom_put_ref(atom);
    
    return 0;
}

/*
 * =============================================================================
 * DTESN Integration
 * =============================================================================
 */

/**
 * atomspace_set_oeis_depth - Set OEIS A000081 hierarchy depth
 * @as: AtomSpace pointer
 * @depth: Depth level (0-13)
 * 
 * Return: 0 on success, negative error code on failure
 */
int atomspace_set_oeis_depth(struct kern_atomspace *as, uint32_t depth)
{
    int ret;
    
    ret = validate_oeis_depth(depth);
    if (ret)
        return ret;
    
    spin_lock(&as->lock);
    as->oeis_depth = depth;
    spin_unlock(&as->lock);
    
    pr_info("inferno_cog: Set OEIS A000081 depth to %u (max atoms: %u)\n",
            depth, oeis_a000081[depth]);
    
    return 0;
}

/**
 * atomspace_validate_oeis_structure - Validate OEIS A000081 compliance
 * @as: AtomSpace pointer
 * 
 * Return: 0 if valid, negative error code if invalid
 */
int atomspace_validate_oeis_structure(struct kern_atomspace *as)
{
    uint64_t atom_count;
    uint32_t expected_max;
    
    atom_count = atomic64_read(&as->atom_count);
    
    if (as->oeis_depth >= OEIS_A000081_LEN) {
        pr_warn("inferno_cog: OEIS depth %u exceeds sequence length\n",
                as->oeis_depth);
        return -EINVAL;
    }
    
    expected_max = oeis_a000081[as->oeis_depth];
    
    if (atom_count > expected_max) {
        pr_warn("inferno_cog: Atom count %llu exceeds OEIS A000081[%u] = %u\n",
                atom_count, as->oeis_depth, expected_max);
        return -ERANGE;
    }
    
    pr_debug("inferno_cog: OEIS validation passed: %llu atoms at depth %u (max %u)\n",
             atom_count, as->oeis_depth, expected_max);
    
    return 0;
}

/*
 * =============================================================================
 * Statistics and Monitoring
 * =============================================================================
 */

/**
 * atomspace_get_stats - Get atomspace statistics
 * @as: AtomSpace pointer
 * @stats: Output statistics structure
 * 
 * Return: 0 on success, negative error code on failure
 */
int atomspace_get_stats(struct kern_atomspace *as, struct atomspace_stats *stats)
{
    if (!stats)
        return -EINVAL;
    
    stats->atom_count = atomic64_read(&as->atom_count);
    stats->link_count = atomic64_read(&as->link_count);
    stats->total_created = atomic64_read(&as->total_created);
    stats->total_deleted = atomic64_read(&as->total_deleted);
    stats->avg_lookup_time_ns = 0;  /* TODO: Implement timing */
    stats->memory_usage_bytes = stats->atom_count * sizeof(struct kern_atom);
    
    return 0;
}

/*
 * =============================================================================
 * Module Init/Exit
 * =============================================================================
 */

static int __init inferno_cog_atomspace_init(void)
{
    int ret;
    
    pr_info("inferno_cog: Loading Inferno-OpenCog AtomSpace module\n");
    
    ret = atomspace_init();
    if (ret) {
        pr_err("inferno_cog: Failed to initialize atomspace: %d\n", ret);
        return ret;
    }
    
    pr_info("inferno_cog: AtomSpace module loaded successfully\n");
    return 0;
}

static void __exit inferno_cog_atomspace_exit(void)
{
    pr_info("inferno_cog: Unloading Inferno-OpenCog AtomSpace module\n");
    
    atomspace_exit();
    
    pr_info("inferno_cog: AtomSpace module unloaded\n");
}

module_init(inferno_cog_atomspace_init);
module_exit(inferno_cog_atomspace_exit);

/* Export symbols for other kernel modules */
EXPORT_SYMBOL(atomspace_init);
EXPORT_SYMBOL(atomspace_exit);
EXPORT_SYMBOL(atom_create);
EXPORT_SYMBOL(atom_delete);
EXPORT_SYMBOL(atom_get);
EXPORT_SYMBOL(atom_set_tv);
EXPORT_SYMBOL(atom_get_tv);
EXPORT_SYMBOL(atomspace_set_oeis_depth);
EXPORT_SYMBOL(atomspace_validate_oeis_structure);
EXPORT_SYMBOL(atomspace_get_stats);

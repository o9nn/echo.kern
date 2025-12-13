/*
 * Inferno-OpenCog ECAN (Economic Attention Networks) Kernel Module
 * =================================================================
 * 
 * Implements attention allocation mechanism as kernel scheduler extension
 * for Echo.Kern DTESN cognitive operating system.
 * 
 * ECAN provides:
 * - Attentional focus management with STI/LTI heaps
 * - Importance spreading algorithms
 * - Economic attention dynamics
 * - Forgetting mechanism
 * - Integration with kernel scheduler for cognitive priority
 * 
 * Copyright (c) 2024 Echo.Kern Development Team
 * Licensed under GPL-2.0
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/time.h>

#include "../../include/dtesn/inferno_cog.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Echo.Kern Development Team");
MODULE_DESCRIPTION("Inferno-OpenCog ECAN Attention Mechanism");
MODULE_VERSION("1.0.0");

/* Global attention bank */
static struct attention_bank global_ecan;

/* Configuration parameters */
static int af_size = ATTENTION_FOCUS_SIZE;
module_param(af_size, int, 0644);
MODULE_PARM_DESC(af_size, "Attentional focus size (default: 1000)");

static int af_rent = 10;
module_param(af_rent, int, 0644);
MODULE_PARM_DESC(af_rent, "Attentional focus rent (default: 10)");

static int forget_threshold = -1000;
module_param(forget_threshold, int, 0644);
MODULE_PARM_DESC(forget_threshold, "Forgetting threshold (default: -1000)");

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

/*
 * =============================================================================
 * Heap Operations (for STI/LTI management)
 * =============================================================================
 */

/* Simple binary heap structure */
struct heap_node {
    atom_id_t atom_id;
    int16_t priority;  /* STI or LTI value */
};

struct heap {
    struct heap_node *nodes;
    uint32_t size;
    uint32_t capacity;
    spinlock_t lock;
};

/* Initialize heap */
static int heap_init(struct heap *h, uint32_t capacity)
{
    h->nodes = kzalloc(capacity * sizeof(struct heap_node), GFP_KERNEL);
    if (!h->nodes)
        return -ENOMEM;
    
    h->size = 0;
    h->capacity = capacity;
    spin_lock_init(&h->lock);
    
    return 0;
}

/* Cleanup heap */
static void heap_destroy(struct heap *h)
{
    if (h->nodes) {
        kfree(h->nodes);
        h->nodes = NULL;
    }
    h->size = 0;
}

/* Insert atom into heap with priority */
static int heap_insert(struct heap *h, atom_id_t atom_id, int16_t priority)
{
    uint32_t i, parent;
    struct heap_node temp;
    
    spin_lock(&h->lock);
    
    if (h->size >= h->capacity) {
        spin_unlock(&h->lock);
        return -ENOMEM;
    }
    
    /* Insert at end */
    i = h->size++;
    h->nodes[i].atom_id = atom_id;
    h->nodes[i].priority = priority;
    
    /* Bubble up */
    while (i > 0) {
        parent = (i - 1) / 2;
        if (h->nodes[i].priority <= h->nodes[parent].priority)
            break;
        
        /* Swap with parent */
        temp = h->nodes[i];
        h->nodes[i] = h->nodes[parent];
        h->nodes[parent] = temp;
        i = parent;
    }
    
    spin_unlock(&h->lock);
    return 0;
}

/* Remove top of heap */
static int heap_pop(struct heap *h, atom_id_t *atom_id, int16_t *priority)
{
    uint32_t i, left, right, largest;
    struct heap_node temp;
    
    spin_lock(&h->lock);
    
    if (h->size == 0) {
        spin_unlock(&h->lock);
        return -ENOENT;
    }
    
    /* Return top */
    if (atom_id)
        *atom_id = h->nodes[0].atom_id;
    if (priority)
        *priority = h->nodes[0].priority;
    
    /* Move last to top */
    h->nodes[0] = h->nodes[--h->size];
    
    /* Bubble down */
    i = 0;
    while (1) {
        left = 2 * i + 1;
        right = 2 * i + 2;
        largest = i;
        
        if (left < h->size && h->nodes[left].priority > h->nodes[largest].priority)
            largest = left;
        if (right < h->size && h->nodes[right].priority > h->nodes[largest].priority)
            largest = right;
        
        if (largest == i)
            break;
        
        /* Swap with largest child */
        temp = h->nodes[i];
        h->nodes[i] = h->nodes[largest];
        h->nodes[largest] = temp;
        i = largest;
    }
    
    spin_unlock(&h->lock);
    return 0;
}

/*
 * =============================================================================
 * ECAN Core Operations
 * =============================================================================
 */

/**
 * ecan_init - Initialize ECAN attention mechanism
 * @as: Associated atomspace
 * 
 * Initializes attention bank with STI/LTI heaps and economic parameters.
 * 
 * Return: 0 on success, negative error code on failure
 */
int ecan_init(struct kern_atomspace *as)
{
    int ret;
    
    pr_info("inferno_cog: Initializing ECAN attention mechanism\n");
    
    /* Initialize STI heap */
    global_ecan.sti_heap = kzalloc(sizeof(struct heap), GFP_KERNEL);
    if (!global_ecan.sti_heap) {
        pr_err("inferno_cog: Failed to allocate STI heap\n");
        return -ENOMEM;
    }
    ret = heap_init(global_ecan.sti_heap, af_size * 2);
    if (ret) {
        kfree(global_ecan.sti_heap);
        return ret;
    }
    
    /* Initialize LTI heap */
    global_ecan.lti_heap = kzalloc(sizeof(struct heap), GFP_KERNEL);
    if (!global_ecan.lti_heap) {
        heap_destroy(global_ecan.sti_heap);
        kfree(global_ecan.sti_heap);
        pr_err("inferno_cog: Failed to allocate LTI heap\n");
        return -ENOMEM;
    }
    ret = heap_init(global_ecan.lti_heap, af_size * 2);
    if (ret) {
        heap_destroy(global_ecan.sti_heap);
        kfree(global_ecan.sti_heap);
        kfree(global_ecan.lti_heap);
        return ret;
    }
    
    /* Initialize parameters */
    global_ecan.af_size = af_size;
    global_ecan.total_sti = 0;
    global_ecan.af_rent = af_rent;
    global_ecan.forget_threshold = forget_threshold;
    global_ecan.last_forget_time_ns = get_timestamp_ns();
    
    spin_lock_init(&global_ecan.lock);
    
    /* Initialize statistics */
    atomic64_set(&global_ecan.stimulations, 0);
    atomic64_set(&global_ecan.spreading_events, 0);
    atomic64_set(&global_ecan.forgetting_events, 0);
    
    pr_info("inferno_cog: ECAN initialized: AF size=%u, rent=%d, threshold=%d\n",
            global_ecan.af_size, global_ecan.af_rent, global_ecan.forget_threshold);
    
    return 0;
}

/**
 * ecan_exit - Cleanup ECAN attention mechanism
 * @ecan: Attention bank to cleanup
 */
void ecan_exit(struct attention_bank *ecan)
{
    pr_info("inferno_cog: Shutting down ECAN\n");
    
    if (ecan->sti_heap) {
        heap_destroy(ecan->sti_heap);
        kfree(ecan->sti_heap);
    }
    
    if (ecan->lti_heap) {
        heap_destroy(ecan->lti_heap);
        kfree(ecan->lti_heap);
    }
    
    pr_info("inferno_cog: ECAN shutdown complete\n");
}

/**
 * ecan_stimulate - Stimulate atom with attention
 * @ecan: Attention bank
 * @id: Atom ID to stimulate
 * @delta: STI delta (positive or negative)
 * 
 * Updates atom's short-term importance (STI) value.
 * 
 * Return: 0 on success, negative error code on failure
 */
int ecan_stimulate(struct attention_bank *ecan, atom_id_t id, int16_t delta)
{
    struct kern_atom *atom;
    int ret;
    int16_t new_sti;
    
    /* Get atom */
    ret = atom_get(id, &atom);
    if (ret)
        return ret;
    
    spin_lock(&ecan->lock);
    
    /* Update STI */
    new_sti = atom->av.sti + delta;
    
    /* Clamp to int16_t range */
    if (new_sti > INT16_MAX)
        new_sti = INT16_MAX;
    if (new_sti < INT16_MIN)
        new_sti = INT16_MIN;
    
    atom->av.sti = new_sti;
    ecan->total_sti += delta;
    
    /* Update STI heap */
    heap_insert(ecan->sti_heap, id, new_sti);
    
    spin_unlock(&ecan->lock);
    
    /* Release atom reference */
    atom_put_ref(atom);
    
    atomic64_inc(&ecan->stimulations);
    
    pr_debug("inferno_cog: Stimulated atom %llu: STI %d -> %d (delta %d)\n",
             id, new_sti - delta, new_sti, delta);
    
    return 0;
}

/**
 * ecan_spread_importance - Spread importance from atom to neighbors
 * @ecan: Attention bank
 * @source: Source atom ID
 * 
 * Implements importance spreading algorithm across atomspace graph.
 * 
 * Return: 0 on success, negative error code on failure
 */
int ecan_spread_importance(struct attention_bank *ecan, atom_id_t source)
{
    struct kern_atom *atom;
    int ret;
    int16_t spread_amount;
    
    /* Get source atom */
    ret = atom_get(source, &atom);
    if (ret)
        return ret;
    
    /* Calculate spread amount (e.g., 10% of STI) */
    spread_amount = atom->av.sti / 10;
    
    /* Release atom reference */
    atom_put_ref(atom);
    
    if (spread_amount == 0) {
        return 0;  /* Nothing to spread */
    }
    
    /* TODO: Implement actual spreading to outgoing/incoming atoms */
    /* For now, this is a stub that will be expanded */
    
    atomic64_inc(&ecan->spreading_events);
    
    pr_debug("inferno_cog: Spreading importance from atom %llu: amount=%d\n",
             source, spread_amount);
    
    return 0;
}

/**
 * ecan_update_af - Update attentional focus
 * @ecan: Attention bank
 * 
 * Updates the attentional focus by applying rent and managing AF size.
 * 
 * Return: 0 on success, negative error code on failure
 */
int ecan_update_af(struct attention_bank *ecan)
{
    atom_id_t atom_id;
    int16_t sti;
    uint32_t count = 0;
    
    spin_lock(&ecan->lock);
    
    /* Apply rent to atoms in attentional focus */
    while (count < ecan->af_size && 
           heap_pop(ecan->sti_heap, &atom_id, &sti) == 0) {
        
        /* Apply rent */
        sti -= ecan->af_rent;
        
        /* Re-insert with reduced STI */
        heap_insert(ecan->sti_heap, atom_id, sti);
        
        count++;
    }
    
    spin_unlock(&ecan->lock);
    
    pr_debug("inferno_cog: Updated AF: processed %u atoms, rent=%d\n",
             count, ecan->af_rent);
    
    return 0;
}

/**
 * ecan_forget - Forget atoms below threshold
 * @ecan: Attention bank
 * 
 * Removes atoms with STI below forgetting threshold.
 * 
 * Return: Number of atoms forgotten
 */
int ecan_forget(struct attention_bank *ecan)
{
    atom_id_t atom_id;
    int16_t sti;
    int forgotten = 0;
    uint64_t now = get_timestamp_ns();
    
    spin_lock(&ecan->lock);
    
    /* Check atoms in STI heap */
    while (heap_pop(ecan->sti_heap, &atom_id, &sti) == 0) {
        if (sti < ecan->forget_threshold) {
            /* Remove from heap first (already done by pop) */
            /* Now safe to delete - no other threads should reference it */
            spin_unlock(&ecan->lock);
            atom_delete(atom_id);
            spin_lock(&ecan->lock);
            forgotten++;
            atomic64_inc(&ecan->forgetting_events);
        } else {
            /* Re-insert - above threshold */
            heap_insert(ecan->sti_heap, atom_id, sti);
            break;
        }
    }
    
    ecan->last_forget_time_ns = now;
    
    spin_unlock(&ecan->lock);
    
    if (forgotten > 0) {
        pr_info("inferno_cog: Forgetting: removed %d atoms below threshold %d\n",
                forgotten, ecan->forget_threshold);
    }
    
    return forgotten;
}

/*
 * =============================================================================
 * Statistics and Monitoring
 * =============================================================================
 */

/**
 * ecan_get_stats - Get ECAN statistics
 * @ecan: Attention bank
 * @stats: Output statistics structure
 * 
 * Return: 0 on success, negative error code on failure
 */
int ecan_get_stats(struct attention_bank *ecan, struct ecan_stats *stats)
{
    if (!stats)
        return -EINVAL;
    
    stats->stimulations = atomic64_read(&ecan->stimulations);
    stats->spreading_events = atomic64_read(&ecan->spreading_events);
    stats->forgetting_events = atomic64_read(&ecan->forgetting_events);
    stats->af_size = ecan->sti_heap ? ecan->sti_heap->size : 0;
    stats->total_sti = ecan->total_sti;
    
    return 0;
}

/*
 * =============================================================================
 * Module Init/Exit
 * =============================================================================
 */

static int __init inferno_cog_ecan_init(void)
{
    int ret;
    
    pr_info("inferno_cog: Loading ECAN attention mechanism module\n");
    
    ret = ecan_init(NULL);  /* Will be associated with atomspace later */
    if (ret) {
        pr_err("inferno_cog: Failed to initialize ECAN: %d\n", ret);
        return ret;
    }
    
    pr_info("inferno_cog: ECAN module loaded successfully\n");
    return 0;
}

static void __exit inferno_cog_ecan_exit(void)
{
    pr_info("inferno_cog: Unloading ECAN module\n");
    
    ecan_exit(&global_ecan);
    
    pr_info("inferno_cog: ECAN module unloaded\n");
}

module_init(inferno_cog_ecan_init);
module_exit(inferno_cog_ecan_exit);

/* Export symbols */
EXPORT_SYMBOL(ecan_init);
EXPORT_SYMBOL(ecan_exit);
EXPORT_SYMBOL(ecan_stimulate);
EXPORT_SYMBOL(ecan_spread_importance);
EXPORT_SYMBOL(ecan_update_af);
EXPORT_SYMBOL(ecan_forget);
EXPORT_SYMBOL(ecan_get_stats);

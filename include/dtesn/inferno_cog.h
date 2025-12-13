/*
 * Inferno-OpenCog Kernel Interface
 * =================================
 * 
 * Kernel-level definitions for OpenCog cognitive primitives implemented
 * as native Inferno kernel services within Echo.Kern DTESN architecture.
 * 
 * This header defines the core data structures and interfaces for:
 * - AtomSpace knowledge representation
 * - ECAN attention allocation
 * - PLN probabilistic reasoning
 * - MOSES evolutionary optimization
 * - 9P cognitive protocol extensions
 * 
 * Copyright (c) 2024 Echo.Kern Development Team
 * Licensed under GPL-2.0
 */

#ifndef _DTESN_INFERNO_COG_H
#define _DTESN_INFERNO_COG_H

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/rbtree.h>
#include <linux/hashtable.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Version information */
#define INFERNO_COG_VERSION_MAJOR    1
#define INFERNO_COG_VERSION_MINOR    0
#define INFERNO_COG_VERSION_PATCH    0

/* Configuration constants */
#define ATOMSPACE_MAX_ATOMS          (1UL << 24)  /* 16M atoms */
#define ATOMSPACE_MAX_NAME_LEN       256
#define ATTENTION_FOCUS_SIZE         1000
#define PLN_MAX_RULES                1000
#define PLN_MAX_INFERENCE_DEPTH      10
#define MOSES_MAX_POPULATION         1000

/*
 * =============================================================================
 * Core Data Types
 * =============================================================================
 */

/* Atom identifier (64-bit unique ID) */
typedef uint64_t atom_id_t;
#define ATOM_ID_INVALID  ((atom_id_t)0)

/* Truth value representation (probabilistic) */
struct truth_value {
    float strength;       /* [0.0, 1.0] - probability/confidence */
    float confidence;     /* [0.0, 1.0] - evidence strength */
    uint32_t count;       /* Evidence count */
};
typedef struct truth_value truth_value_t;

/* Attention value (ECAN) */
struct attention_value {
    int16_t sti;          /* Short-term importance [-32768, 32767] */
    int16_t lti;          /* Long-term importance [-32768, 32767] */
    int16_t vlti;         /* Very long-term importance */
};
typedef struct attention_value attention_value_t;

/* Atom types (OpenCog compatible) */
enum atom_type {
    /* Node types */
    ATOM_TYPE_NODE           = 0x0000,
    ATOM_TYPE_CONCEPT        = 0x0001,
    ATOM_TYPE_PREDICATE      = 0x0002,
    ATOM_TYPE_VARIABLE       = 0x0003,
    ATOM_TYPE_SCHEMA         = 0x0004,
    
    /* Link types */
    ATOM_TYPE_LINK           = 0x1000,
    ATOM_TYPE_INHERITANCE    = 0x1001,
    ATOM_TYPE_SIMILARITY     = 0x1002,
    ATOM_TYPE_IMPLICATION    = 0x1003,
    ATOM_TYPE_EVALUATION     = 0x1004,
    ATOM_TYPE_MEMBER         = 0x1005,
    ATOM_TYPE_LIST           = 0x1006,
    ATOM_TYPE_AND            = 0x1007,
    ATOM_TYPE_OR             = 0x1008,
    ATOM_TYPE_NOT            = 0x1009,
    
    /* Special types */
    ATOM_TYPE_BIND           = 0x2000,
    ATOM_TYPE_PATTERN        = 0x2001,
};
typedef enum atom_type atom_type_t;

/*
 * =============================================================================
 * AtomSpace Structures
 * =============================================================================
 */

/* Atom structure (kernel representation) */
struct kern_atom {
    atom_id_t id;                    /* Unique identifier */
    atom_type_t type;                /* Atom type */
    char name[ATOMSPACE_MAX_NAME_LEN]; /* Name (for nodes) */
    
    truth_value_t tv;                /* Truth value */
    attention_value_t av;            /* Attention value */
    
    /* Link-specific fields */
    atom_id_t *outgoing;            /* Outgoing set (for links) */
    uint32_t outgoing_size;         /* Size of outgoing set */
    
    /* Graph connectivity */
    struct list_head incoming;       /* Incoming set */
    
    /* Tree structure (OEIS A000081) */
    uint32_t depth;                  /* Tree depth */
    struct kern_atom *parent;        /* Parent in hierarchy */
    
    /* Kernel metadata */
    struct rb_node rb_node;          /* Red-black tree node */
    struct hlist_node hash_node;     /* Hash table node */
    atomic_t refcount;               /* Reference count */
    uint64_t creation_time_ns;       /* Creation timestamp */
};

/* Kernel AtomSpace */
struct kern_atomspace {
    /* Core data structures */
    struct rb_root atoms;            /* Red-black tree by ID */
    DECLARE_HASHTABLE(atom_index, 10); /* Hash table by name */
    
    /* Concurrency control */
    spinlock_t lock;                 /* Global atomspace lock */
    
    /* ID generation */
    atomic64_t next_atom_id;         /* Atomic ID counter */
    
    /* Statistics */
    atomic64_t atom_count;           /* Current atom count */
    atomic64_t link_count;           /* Current link count */
    atomic64_t total_created;        /* Total atoms created */
    atomic64_t total_deleted;        /* Total atoms deleted */
    
    /* ECAN integration */
    struct attention_bank *ecan;     /* Attention mechanism */
    
    /* DTESN integration */
    uint32_t oeis_depth;             /* Current OEIS A000081 depth */
    struct membrane *security_membrane; /* P-System security */
    struct esn_reservoir *reservoir;  /* ESN temporal dynamics */
};

/*
 * =============================================================================
 * ECAN Attention Mechanism
 * =============================================================================
 */

/* Attention bank (kernel) */
struct attention_bank {
    /* Attentional focus */
    struct heap *sti_heap;           /* STI-ordered atoms */
    struct heap *lti_heap;           /* LTI-ordered atoms */
    uint32_t af_size;                /* Attentional focus size */
    
    /* Importance spreading */
    int16_t total_sti;               /* Total STI in system */
    int16_t af_rent;                 /* Attentional focus rent */
    
    /* Forgetting mechanism */
    int16_t forget_threshold;        /* Forgetting threshold */
    uint64_t last_forget_time_ns;    /* Last forgetting time */
    
    /* Concurrency */
    spinlock_t lock;
    
    /* Statistics */
    atomic64_t stimulations;         /* Total stimulations */
    atomic64_t spreading_events;     /* Importance spreading events */
    atomic64_t forgetting_events;    /* Forgetting events */
};

/*
 * =============================================================================
 * PLN Inference Engine
 * =============================================================================
 */

/* Inference rule */
struct inference_rule {
    uint32_t rule_id;                /* Unique rule ID */
    char name[64];                   /* Rule name */
    
    /* Pattern matching */
    atom_id_t *premise_pattern;      /* Premise pattern */
    uint32_t premise_count;          /* Number of premises */
    
    atom_id_t conclusion_pattern;    /* Conclusion pattern */
    
    /* Rule application */
    truth_value_t (*apply)(truth_value_t *premise_tvs, uint32_t count);
    
    /* Metadata */
    float weight;                    /* Rule weight/priority */
    uint32_t application_count;      /* Times applied */
    struct list_head list;           /* List node */
};

/* Forward chainer */
struct forward_chainer {
    struct kern_atomspace *as;       /* Target atomspace */
    struct list_head rule_base;      /* Available rules */
    uint32_t max_iterations;         /* Maximum iterations */
    
    /* Statistics */
    atomic64_t total_inferences;     /* Total inferences made */
    atomic64_t successful_inferences; /* Successful inferences */
};

/* Backward chainer */
struct backward_chainer {
    struct kern_atomspace *as;       /* Target atomspace */
    struct list_head rule_base;      /* Available rules */
    uint32_t max_depth;              /* Maximum backward chaining depth */
    
    /* Statistics */
    atomic64_t total_queries;        /* Total backward queries */
    atomic64_t successful_queries;   /* Successful queries */
};

/* Unification engine */
struct unification_engine {
    struct kern_atomspace *as;       /* Target atomspace */
    
    /* Unification cache */
    struct hash_table *unify_cache;  /* Cache for unification results */
    
    /* Statistics */
    atomic64_t unifications;         /* Total unifications */
    atomic64_t cache_hits;           /* Cache hits */
};

/* PLN engine */
struct pln_engine {
    struct kern_atomspace *as;       /* Associated atomspace */
    
    /* Chaining engines */
    struct forward_chainer fc;       /* Forward chainer */
    struct backward_chainer bc;      /* Backward chainer */
    struct unification_engine ue;    /* Unification engine */
    
    /* Rule management */
    struct list_head rules;          /* All inference rules */
    uint32_t rule_count;             /* Number of rules */
    
    /* Concurrency */
    spinlock_t lock;
    
    /* Statistics */
    atomic64_t total_inferences;     /* Total inference operations */
};

/*
 * =============================================================================
 * MOSES Evolutionary Optimization
 * =============================================================================
 */

/* Program representation (tree-based) */
struct program {
    atom_id_t root;                  /* Root atom of program */
    struct kern_atomspace *as;       /* Associated atomspace */
    float fitness;                   /* Fitness score */
    uint32_t complexity;             /* Program complexity */
};

/* Population */
struct moses_population {
    struct program *programs;        /* Array of programs */
    uint32_t size;                   /* Population size */
    uint32_t max_size;               /* Maximum size */
    
    /* Fitness tracking */
    float best_fitness;              /* Best fitness in population */
    uint32_t best_index;             /* Index of best program */
    
    /* Statistics */
    atomic64_t evaluations;          /* Total fitness evaluations */
};

/* MOSES optimizer */
struct moses_optimizer {
    struct kern_atomspace *as;       /* Target atomspace */
    struct moses_population pop;     /* Current population */
    
    /* Optimization parameters */
    uint32_t max_generations;        /* Maximum generations */
    float mutation_rate;             /* Mutation probability */
    float crossover_rate;            /* Crossover probability */
    
    /* Fitness function */
    float (*fitness_fn)(struct program *p, void *data);
    void *fitness_data;
    
    /* Concurrency */
    spinlock_t lock;
    
    /* Statistics */
    atomic64_t generations;          /* Generations evolved */
    atomic64_t total_evaluations;    /* Total fitness evaluations */
};

/*
 * =============================================================================
 * 9P Cognitive Protocol Extensions
 * =============================================================================
 */

/* Cognitive operation flags */
#define COG_FLAG_ASYNC           (1 << 0)  /* Asynchronous operation */
#define COG_FLAG_DISTRIBUTED     (1 << 1)  /* Distributed operation */
#define COG_FLAG_CACHED          (1 << 2)  /* Use cache if available */
#define COG_FLAG_ATTENTION       (1 << 3)  /* Update attention values */

/* Cognitive message types (9P extensions) */
enum cog_msg_type {
    COG_MSG_ATOM_CREATE      = 200,
    COG_MSG_ATOM_DELETE      = 201,
    COG_MSG_ATOM_GET         = 202,
    COG_MSG_PATTERN_MATCH    = 203,
    COG_MSG_INFER            = 204,
    COG_MSG_ATTENTION_UPDATE = 205,
    COG_MSG_MOSES_EVOLVE     = 206,
};

/* Cognitive request header */
struct cog_request {
    uint16_t msg_type;               /* Cognitive message type */
    uint32_t flags;                  /* Operation flags */
    uint64_t context_id;             /* Context identifier */
    uint32_t data_len;               /* Data length */
    uint8_t data[];                  /* Variable-length data */
} __attribute__((packed));

/* Cognitive response header */
struct cog_response {
    uint16_t msg_type;               /* Response type */
    uint32_t status;                 /* Status code */
    truth_value_t tv;                /* Truth value (if applicable) */
    attention_value_t av;            /* Attention value (if applicable) */
    uint32_t data_len;               /* Response data length */
    uint8_t data[];                  /* Variable-length data */
} __attribute__((packed));

/*
 * =============================================================================
 * Kernel API Functions
 * =============================================================================
 */

/* AtomSpace operations */
int atomspace_init(void);
void atomspace_exit(void);
atom_id_t atom_create(atom_type_t type, const char *name, truth_value_t tv);
int atom_delete(atom_id_t id);
int atom_get(atom_id_t id, struct kern_atom **out);
int atom_set_tv(atom_id_t id, truth_value_t tv);
int atom_get_tv(atom_id_t id, truth_value_t *out);

/* Link operations */
atom_id_t link_create(atom_type_t type, atom_id_t *targets, uint32_t n, 
                      truth_value_t tv);
int link_get_outgoing(atom_id_t link_id, atom_id_t **out, uint32_t *n);
int link_get_incoming(atom_id_t atom_id, atom_id_t **out, uint32_t *n);

/* Pattern matching */
int pattern_match(struct kern_atomspace *as, atom_id_t pattern, 
                 atom_id_t **results, uint32_t *n);
int bind_link(struct kern_atomspace *as, atom_id_t pattern, atom_id_t action,
             atom_id_t **results, uint32_t *n);

/* ECAN operations */
int ecan_init(struct kern_atomspace *as);
void ecan_exit(struct attention_bank *ecan);
int ecan_stimulate(struct attention_bank *ecan, atom_id_t id, int16_t delta);
int ecan_spread_importance(struct attention_bank *ecan, atom_id_t source);
int ecan_update_af(struct attention_bank *ecan);
int ecan_forget(struct attention_bank *ecan);

/* PLN operations */
int pln_init(struct kern_atomspace *as);
void pln_exit(struct pln_engine *pln);
int pln_add_rule(struct pln_engine *pln, struct inference_rule *rule);
int pln_infer(struct pln_engine *pln, atom_id_t *premises, uint32_t n,
             atom_id_t *conclusion);
int pln_forward_chain(struct pln_engine *pln, atom_id_t seed, 
                     atom_id_t **results, uint32_t *n);
int pln_backward_chain(struct pln_engine *pln, atom_id_t goal,
                      atom_id_t **results, uint32_t *n);

/* MOSES operations */
int moses_init(struct kern_atomspace *as);
void moses_exit(struct moses_optimizer *moses);
int moses_evolve(struct moses_optimizer *moses, struct program *seed,
                uint32_t generations, struct program *result);
int moses_step(struct moses_optimizer *moses);

/* 9P cognitive operations */
int cog_handle_request(struct cog_request *req, struct cog_response **resp);
int cog_atom_create_msg(struct cog_request *req, struct cog_response **resp);
int cog_pattern_match_msg(struct cog_request *req, struct cog_response **resp);
int cog_infer_msg(struct cog_request *req, struct cog_response **resp);

/*
 * =============================================================================
 * DTESN Integration
 * =============================================================================
 */

/* Map atomspace to OEIS A000081 hierarchy */
int atomspace_set_oeis_depth(struct kern_atomspace *as, uint32_t depth);
int atomspace_validate_oeis_structure(struct kern_atomspace *as);

/* P-System membrane integration */
int atomspace_set_membrane(struct kern_atomspace *as, struct membrane *m);
int atomspace_transfer_atom(atom_id_t id, struct membrane *target);

/* ESN reservoir integration */
int atomspace_set_reservoir(struct kern_atomspace *as, struct esn_reservoir *r);
int atomspace_update_dynamics(struct kern_atomspace *as);

/*
 * =============================================================================
 * Statistics and Monitoring
 * =============================================================================
 */

/* AtomSpace statistics */
struct atomspace_stats {
    uint64_t atom_count;             /* Current atoms */
    uint64_t link_count;             /* Current links */
    uint64_t total_created;          /* Total created */
    uint64_t total_deleted;          /* Total deleted */
    uint64_t avg_lookup_time_ns;     /* Average lookup time */
    uint64_t memory_usage_bytes;     /* Memory usage */
};

/* ECAN statistics */
struct ecan_stats {
    uint64_t stimulations;           /* Total stimulations */
    uint64_t spreading_events;       /* Spreading events */
    uint64_t forgetting_events;      /* Forgetting events */
    uint32_t af_size;                /* Current AF size */
    int16_t total_sti;               /* Total STI */
};

/* PLN statistics */
struct pln_stats {
    uint64_t total_inferences;       /* Total inferences */
    uint64_t successful_inferences;  /* Successful inferences */
    uint64_t rule_applications;      /* Rule applications */
    uint64_t avg_inference_time_ns;  /* Average inference time */
};

/* Get statistics */
int atomspace_get_stats(struct kern_atomspace *as, struct atomspace_stats *stats);
int ecan_get_stats(struct attention_bank *ecan, struct ecan_stats *stats);
int pln_get_stats(struct pln_engine *pln, struct pln_stats *stats);

#ifdef __cplusplus
}
#endif

#endif /* _DTESN_INFERNO_COG_H */

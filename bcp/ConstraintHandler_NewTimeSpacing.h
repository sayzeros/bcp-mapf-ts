#ifndef MAPF_CONSTRAINTHANDLER_NEWTIMESPACING_H
#define MAPF_CONSTRAINTHANDLER_NEWTIMESPACING_H

#ifdef USE_NEW_TIME_SPACING

#include "Includes.h"
#include "Coordinates.h"
#include "ProblemData.h"

struct NewTimeSpacing
{
    SCIP_ROW* row;          // LP row
};

// Create the constraint handler for vertex conflicts and include it
SCIP_RETCODE SCIPincludeConshdlrNewTimeSpacing(
    SCIP* scip    // SCIP
);

// Create a constraint for vertex conflicts and include it
SCIP_RETCODE SCIPcreateConsNewTimeSpacing(
    SCIP* scip,                 // SCIP
    SCIP_CONS** cons,           // Pointer to hold the created constraint
    const char* name,           // Name of constraint
    SCIP_Bool initial,          // Should the LP relaxation of constraint be in the initial LP?
    SCIP_Bool separate,         // Should the constraint be separated during LP processing?
    SCIP_Bool enforce,          // Should the constraint be enforced during node processing?
    SCIP_Bool check,            // Should the constraint be checked for feasibility?
    SCIP_Bool propagate,        // Should the constraint be propagated during node processing?
    SCIP_Bool local,            // Is constraint only valid locally?
    SCIP_Bool modifiable,       // Is constraint modifiable (subject to column generation)?
    SCIP_Bool dynamic,          // Is constraint subject to aging?
    SCIP_Bool removable,        // Should the relaxation be removed from the LP due to aging or cleanup?
    SCIP_Bool stickingatnode    // Should the constraint always be kept at the node where it was added, even
                                // if it may be moved to a more global node?
);

SCIP_RETCODE new_time_spacing_add_var(
    SCIP* scip,                // SCIP
    SCIP_CONS* cons,           // Vertex conflicts constraint
    SCIP_VAR* var,             // Variable
    const Time path_length,    // Path length
    const Edge* const path     // Path
);

namespace TruffleHog
{

union NodeTimeAgentSpace
{
    struct
    {
        Node n : 32;
        Time t : 16;
        Agent a : 8;
        int h : 8;
    };
    uint64_t ntah;

    NodeTimeAgentSpace() noexcept = default;
    NodeTimeAgentSpace(const uint64_t ntah) noexcept : ntah(ntah) {}
    explicit NodeTimeAgentSpace(const Node n, const Time t, const Agent a, const int h) noexcept : n(n), t(t), a(a), h(h) {}
};
static_assert(std::is_trivial<NodeTimeAgentSpace>::value);
inline bool operator==(const NodeTimeAgentSpace a, const NodeTimeAgentSpace b)
{
    return a.ntah == b.ntah;
}
inline bool operator!=(const NodeTimeAgentSpace a, const NodeTimeAgentSpace b)
{
    return a.ntah != b.ntah;
}
}


namespace robin_hood
{
template<>
struct hash<TruffleHog::NodeTimeAgentSpace>
{
    inline std::size_t operator()(const TruffleHog::NodeTimeAgentSpace ntah) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(ntah.ntah);
    }
};
}


const HashTable<NodeTimeAgentSpace, NewTimeSpacing>& new_time_spacing_get_constraints(
    SCIP_ProbData* probdata    // Problem data
);

#endif

#endif
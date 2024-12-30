#ifndef MAPF_CONSTRAINTHANDLER_OLDTIMESPACING_H
#define MAPF_CONSTRAINTHANDLER_OLDTIMESPACING_H

#include "Includes.h"
#include "Coordinates.h"
#include "ProblemData.h"

struct OldTimeSpacing
{
    SCIP_ROW* row;          // LP row
};

// Create the constraint handler for vertex conflicts and include it
SCIP_RETCODE SCIPincludeConshdlrOldTimeSpacing(
    SCIP* scip    // SCIP
);

// Create a constraint for vertex conflicts and include it
SCIP_RETCODE SCIPcreateConsOldTimeSpacing(
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

SCIP_RETCODE old_time_spacing_add_var(
    SCIP* scip,                // SCIP
    SCIP_CONS* cons,           // Vertex conflicts constraint
    SCIP_VAR* var,             // Variable
    const Time path_length,    // Path length
    const Edge* const path     // Path
);

namespace TruffleHog
{

union NodeTimeAgent
{
    struct
    {
        Node n;
        Time t;
        Agent a;
    };
    uint64_t nta;

    NodeTimeAgent() noexcept = default;
    NodeTimeAgent(const uint64_t nta) noexcept : nta(nta) {}
    explicit NodeTimeAgent(const Node n, const Time t, const Agent a) noexcept : n(n), t(t), a(a) {}
};
// static_assert(sizeof(NodeTimeAgent) == 12);
static_assert(std::is_trivial<NodeTimeAgent>::value);
inline bool operator==(const NodeTimeAgent a, const NodeTimeAgent b)
{
    return a.nta == b.nta;
}
inline bool operator!=(const NodeTimeAgent a, const NodeTimeAgent b)
{
    return a.nta != b.nta;
}

}

template<>
struct robin_hood::hash<TruffleHog::NodeTimeAgent>
{
    inline std::size_t operator()(const TruffleHog::NodeTimeAgent nta) const noexcept
    {
        return robin_hood::hash<uint64_t>{}(nta.nta);
    }
};

template<>
struct fmt::formatter<TruffleHog::NodeTimeAgent>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    inline auto format(const TruffleHog::NodeTimeAgent nta, FormatContext& ctx)
    {
        return format_to(ctx.out(), "(n={},t={},a={})", nta.n, nta.t, nta.a);
    }
};

const HashTable<NodeTimeAgent, OldTimeSpacing>& old_time_spacing_get_constraints(
    SCIP_ProbData* probdata    // Problem data
);

#endif

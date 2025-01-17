//#define PRINT_DEBUG

#ifdef USE_OLD_TIME_SPACING

#include "ConstraintHandler_OldTimeSpacing.h"
#include "ProblemData.h"
#include "VariableData.h"

#define CONSHDLR_NAME          "old_time_spacing"
#define CONSHDLR_DESC          "Constraint handler for old time spacing"
#define CONSHDLR_SEPAPRIORITY  9         // priority of the constraint handler for separation
#define CONSHDLR_ENFOPRIORITY  -1100000    // priority of the constraint handler for constraint enforcing
#define CONSHDLR_CHECKPRIORITY -1100000    // priority of the constraint handler for checking feasibility
#define CONSHDLR_SEPAFREQ      1          // frequency for separating cuts; zero means to separate only in the root node
#define CONSHDLR_EAGERFREQ     1          // frequency for using all instead of only the useful constraints in separation,
                                          // propagation and enforcement, -1 for no eager evaluations, 0 for first only
#define CONSHDLR_DELAYSEPA     TRUE       // should separation method be delayed, if other separators found cuts?
#define CONSHDLR_NEEDSCONS     TRUE       // should the constraint handler be skipped, if no constraints are available?

// Data for old time spacing
struct OldTimeSpacingConsData
{
    HashTable<NodeTimeAgent, OldTimeSpacing> conflicts;
};

// Create a constraint for old time spacing and include it
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
)
{
    // Find constraint handler.
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME);
    release_assert(conshdlr, "Constraint handler for old time spacing is not found");

    // Create constraint data.
    OldTimeSpacingConsData* consdata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &consdata));
    debug_assert(consdata);
    new(consdata) OldTimeSpacingConsData;
    consdata->conflicts.reserve(5000);

    // Create constraint.
    SCIP_CALL(SCIPcreateCons(scip,
                             cons,
                             name,
                             conshdlr,
                             reinterpret_cast<SCIP_CONSDATA*>(consdata),
                             initial,
                             separate,
                             enforce,
                             check,
                             propagate,
                             local,
                             modifiable,
                             dynamic,
                             removable,
                             stickingatnode));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE old_time_spacing_create_cut(
    SCIP* scip,                                        // SCIP
    SCIP_CONS* cons,                                   // Constraint
    OldTimeSpacingConsData* consdata,                 // Constraint data
    const NodeTimeAgent nta,                                 // Node-time of the conflict
    const Vector<Pair<SCIP_VAR*, SCIP_Real>>& vars,    // Variables
    SCIP_Result* result                                // Output result
)
{
    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto N = SCIPprobdataGetN(probdata);
    const auto& agents = SCIPprobdataGetAgentsData(probdata);
    auto ts = SCIPprobdataGetTimeSpacing(probdata);

    // Create constraint name.
#ifdef DEBUG
    const auto& map = SCIPprobdataGetMap(probdata);
    const auto [x, y] = map.get_xy(nta.n);
    const auto name = fmt::format("old_time_spacing(({},{}),{},{})", x, y, nta.t, nta.a);
#endif

    // Create a row.
    SCIP_ROW* row = nullptr;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip,
                                     &row,
                                     cons,
#ifdef DEBUG
                                     name.c_str(),
#else
                                     "",
#endif
                                     -SCIPinfinity(scip),
                                     1.0,
                                     FALSE,
                                     TRUE,
                                     FALSE));
    debug_assert(row);

    // Add variables to the constraint.
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
#ifdef DEBUG
    SCIP_Real lhs = 0.0;
#endif
    for (const auto& [var, var_val] : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);
        const auto a = SCIPvardataGetAgent(vardata);
        debug_assert(var_val == SCIPgetSolVal(scip, nullptr, var));

        // Add coefficients.
        if  (nta.a == a && path[nta.t].n == nta.n)
        {
            // Add the coefficient.
            SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
        }
        else if (nta.a != a)
        {
          for (int h = 0; h <= ts; h++)
          {
            if (nta.t + h < path_length && path[nta.t + h].n == nta.n)
            {
              // Add the coefficient.
              SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0 / ((double) ts + 1)));
            }
          }
        }
    }
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
#ifdef DEBUG
    debug_assert(SCIPisSumGT(scip, lhs, 1.0 - 1e-6));
#endif

    // Add the row to the LP.
    SCIP_Bool infeasible;
    SCIP_CALL(SCIPaddRow(scip, row, true, &infeasible));

    // Set status.
    if (infeasible)
    {
        *result = SCIP_CUTOFF;
    }
    else
    {
        *result = SCIP_SEPARATED;
    }

    // Store the constraint.
    debug_assert(consdata->conflicts.find(nta) == consdata->conflicts.end());
    consdata->conflicts[nta] = {row};

    // Done.
    return SCIP_OKAY;
}

// Checker (check whether the solution violates the old_time_spacing constraints or not)
static
SCIP_RETCODE old_time_spacing_check(
    SCIP* scip,            // SCIP
    SCIP_SOL* sol,         // Solution
    SCIP_RESULT* result    // Pointer to store the result
)
{
    // Print.
    debugln("Starting checker for old time spacing on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, sol));

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);
    const auto ts = SCIPprobdataGetTimeSpacing(probdata);
    const auto N = SCIPprobdataGetN(probdata);

    // Calculate the LHS of old_time_spacing constraints by summing the columns.
    HashTable<NodeTimeAgent, SCIP_Real> lhs;
    for (const auto& [var, _] : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);
        const auto a = SCIPvardataGetAgent(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Sum lhs value.
        if (SCIPisPositive(scip, var_val))
        {
            for (Time t = 0; t < path_length; ++t)
            {
                const NodeTimeAgent nta{path[t].n, t, a};
                lhs[nta] += var_val;
                for (Agent aa = 0; aa < N; aa++)
                {
                  if (aa == a) continue;
                  for (int h = 0; h <= ts; h++)
                  {
                    // if (t - h >= 0)
                    {
                      const NodeTimeAgent nta{path[t].n, t - h, aa};
                      lhs[nta] += var_val / ((double) ts + 1);
                    }
                  }
                }
            }
        }
    }

    // Check for conflicts.
    for (const auto [nt, val] : lhs)
        if (SCIPisSumGT(scip, val, 1.0))
        {
            // Infeasible.
            *result = SCIP_INFEASIBLE;
            return SCIP_OKAY;
        }

    // Done.
    return SCIP_OKAY;
}

// Separator
static
SCIP_RETCODE old_time_spacing_separate(
    SCIP* scip,                 // SCIP
    SCIP_CONS* cons,            // Constraint
    SCIP_SOL* sol,              // Solution
    SCIP_RESULT* result         // Pointer to store the result
)
{
    // Print.
    debugln("Starting separator for old time spacing on solution with obj {:.6f}:",
            SCIPgetSolOrigObj(scip, nullptr));

    // Print paths.
#ifdef PRINT_DEBUG
    print_used_paths(scip);
#endif

    // Get constraint data.
    auto consdata = reinterpret_cast<OldTimeSpacingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);
    const auto ts = SCIPprobdataGetTimeSpacing(probdata);
    const auto N = SCIPprobdataGetN(probdata);

    // Update variable values.
    update_variable_values(scip);

    // Get variables.
    const auto& vars = SCIPprobdataGetVars(probdata);

    // Calculate the lhs of old_time_spacing constraints by summing the columns.
    HashTable<NodeTimeAgent, SCIP_Real> lhs;
    // println("lhs[144958, 387, 0] = {}", lhs[NodeTimeAgent{144958, 387, 0}]);
    for (const auto& [var, _] : vars)
    {
        // Get the path.
        debug_assert(var);
        auto vardata = SCIPvarGetData(var);
        const auto path_length = SCIPvardataGetPathLength(vardata);
        const auto path = SCIPvardataGetPath(vardata);
        const auto a = SCIPvardataGetAgent(vardata);

        // Get the variable value.
        const auto var_val = SCIPgetSolVal(scip, sol, var);

        // Sum lhs value.
        if (SCIPisPositive(scip, var_val))
        {
            // println("b lhs[144958, 387, 0] = {}", lhs[NodeTimeAgent{144958, 387, 0}]);
            for (Time t = 0; t < path_length; ++t)
            {
                const NodeTimeAgent nta{path[t].n, t, a};
                lhs[nta] += var_val;
                // if (nta.n == 144958 && nta.t == 387 && nta.a == 0)
                // {
                //   println("added {} {} {}  : {}  {}", nta.n, nta.t, nta.a, var_val, lhs[nta]);
                // }
                for (Agent aa = 0; aa < N; aa++)
                {
                  if (aa == a) continue;
                  for (int h = 0; h <= ts; h++)
                  {
                      const NodeTimeAgent nta2{path[t].n, t - h, aa};
                      lhs[nta2] += var_val / ((double) ts + 1);
                      // if (nta2.n == 144958 && nta2.t == 387 && nta2.a == 0)
                      // {
                      //   println("added {} {} {}  : {}  {}", nta2.n, nta2.t, nta2.a, var_val / ((double) ts + 1), lhs[nta2]);
                      // }
                      // if (robin_hood::hash<uint64_t>{}(nta2.nta) == robin_hood::hash<uint64_t>{}(NodeTimeAgent{144958, 387, 0}.nta))
                      // {
                      //   println("fuck {} {} {}  : {}  {}", nta2.n, nta2.t, nta2.a, var_val / ((double) ts + 1), lhs[nta2]);
                      // }
                  }
                }
            }
            // println("a lhs[144958, 387, 0] = {}", lhs[NodeTimeAgent{144958, 387, 0}]);
        }
    }

    // Create cuts.
    // println("lhs[144958, 387, 0] = {}", lhs[NodeTimeAgent{144958, 387, 0}]);
    for (const auto [nta, val] : lhs)
        if (SCIPisSumGT(scip, val, 1.0))
        {
            // Reactive the cut if it already exists. Otherwise create the cut.
            if (auto it = consdata->conflicts.find(nta); it != consdata->conflicts.end())
            {
                // Reactivate the row if it is not in the LP.
                const auto& [row] = it->second;
                if (!SCIProwIsInLP(row))
                {
                    SCIP_Bool infeasible;
                    SCIP_CALL(SCIPaddRow(scip, row, true, &infeasible));
                    *result = SCIP_SEPARATED;
                }
                else
                {
                    println("old_time_spacing ctrs {} {} {}  : {}", nta.n, nta.t, nta.a, val);
                    release_assert(SCIPisSumLE(scip, val, 1.0 + 1e-6),
                                   "Old time spacing conflict constraint is violated but is already active");
                }
            }
            else
            {
                // Create cut.
                SCIP_CALL(old_time_spacing_create_cut(scip, cons, consdata, nta, vars, result));
            }
        }

    // Done.
    return SCIP_OKAY;
}

// Copy method for constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSHDLRCOPY(conshdlrCopyOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(valid);

    // Include constraint handler.
    SCIP_CALL(SCIPincludeConshdlrOldTimeSpacing(scip));

    // Done.
    *valid = TRUE;
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free constraint data
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSDELETE(consDeleteOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);
    debug_assert(consdata);
    debug_assert(*consdata);

    // Override type.
    auto consdata2 = reinterpret_cast<OldTimeSpacingConsData**>(consdata);

    // Free memory.
    (*consdata2)->~OldTimeSpacingConsData();
    SCIPfreeBlockMemory(scip, consdata2);

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Free rows
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSEXITSOL(consExitsolOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(nconss == 0 || conss);

    // Loop through all constraints.
    for (Int c = 0; c < nconss; ++c)
    {
        // Get constraint.
        auto cons = conss[c];
        debug_assert(cons);

        // Get constraint data.
        auto consdata = reinterpret_cast<OldTimeSpacingConsData*>(SCIPconsGetData(cons));
        debug_assert(consdata);

        // Free row for each vertex conflict.
        for (auto& [nt, old_time_spacing] : consdata->conflicts)
        {
            auto& [row] = old_time_spacing;
            SCIP_CALL(SCIPreleaseRow(scip, &row));
        }
        consdata->conflicts.clear();
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Transform constraint data into data belonging to the transformed problem
static
SCIP_DECL_CONSTRANS(consTransOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(sourcecons);
    debug_assert(targetcons);

    // Get data of original constraint.
    auto sourcedata =
        reinterpret_cast<OldTimeSpacingConsData*>(SCIPconsGetData(sourcecons));
    debug_assert(sourcedata);

    // Create constraint data.
    OldTimeSpacingConsData* targetdata = nullptr;
    SCIP_CALL(SCIPallocBlockMemory(scip, &targetdata));
    debug_assert(targetdata);
    new(targetdata) OldTimeSpacingConsData(*sourcedata);

    // Must begin with no old time spacing.
    release_assert(sourcedata->conflicts.empty(),
                   "old time spacing ctrs exist in original problem before transformation");

    // Create constraint.
    char name[SCIP_MAXSTRLEN];
    SCIPsnprintf(name, SCIP_MAXSTRLEN, "t_%s", SCIPconsGetName(sourcecons));
    SCIP_CALL(SCIPcreateCons(scip,
                             targetcons,
                             name,
                             conshdlr,
                             reinterpret_cast<SCIP_CONSDATA*>(targetdata),
                             SCIPconsIsInitial(sourcecons),
                             SCIPconsIsSeparated(sourcecons),
                             SCIPconsIsEnforced(sourcecons),
                             SCIPconsIsChecked(sourcecons),
                             SCIPconsIsPropagated(sourcecons),
                             SCIPconsIsLocal(sourcecons),
                             SCIPconsIsModifiable(sourcecons),
                             SCIPconsIsDynamic(sourcecons),
                             SCIPconsIsRemovable(sourcecons),
                             SCIPconsIsStickingAtNode(sourcecons)));

    // Done.
    return SCIP_OKAY;
}

// Feasibility check method for integral solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSCHECK(consCheckOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(nconss == 0 || conss);
    debug_assert(result);

    // Start.
    *result = SCIP_FEASIBLE;

    // Start checker.
    debug_assert(sol);
    SCIP_CALL(old_time_spacing_check(scip, sol, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Constraint enforcing method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSENFOLP(consEnfolpOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_FEASIBLE;

    // Get constraint.
    debug_assert(nconss == 1);
    auto cons = conss[0];
    debug_assert(cons);

    // Start separator.
    SCIP_CALL(old_time_spacing_separate(scip, cons, nullptr, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Constraint enforcing method for pseudo solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSENFOPS(consEnfopsOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_FEASIBLE;

    // Start checker.
    SCIP_CALL(old_time_spacing_check(scip, nullptr, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for LP solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSSEPALP(consSepalpOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Get constraint.
    debug_assert(nconss == 1);
    auto cons = conss[0];
    debug_assert(cons);

    // Start separator.
    SCIP_CALL(old_time_spacing_separate(scip, cons, nullptr, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Separation method for arbitrary primal solutions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSSEPASOL(consSepasolOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(conss);
    debug_assert(result);

    // Start.
    *result = SCIP_DIDNOTFIND;

    // Get constraint.
    debug_assert(nconss == 1);
    auto cons = conss[0];
    debug_assert(cons);

    // Start separator.
    debug_assert(sol);
    SCIP_CALL(old_time_spacing_separate(scip, cons, sol, result));

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Variable rounding lock method of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSLOCK(consLockOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(conshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);

    // Get problem data.
    auto probdata = SCIPgetProbData(scip);

    // Lock rounding of variables. (Round up may invalidate the constraint.)
    const auto& vars = SCIPprobdataGetVars(probdata);
    for (const auto& [var, _] : vars)
    {
        debug_assert(var);
        SCIP_CALL(SCIPaddVarLocksType(scip, var, locktype, nlocksneg, nlockspos));
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Copying constraint of constraint handler
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static
SCIP_DECL_CONSCOPY(consCopyOldTimeSpacing)
{
    // Check.
    debug_assert(scip);
    debug_assert(sourceconshdlr);
    debug_assert(strcmp(SCIPconshdlrGetName(sourceconshdlr), CONSHDLR_NAME) == 0);
    debug_assert(cons);
    debug_assert(sourcescip);
    debug_assert(sourcecons);
    debug_assert(varmap);

    // Stop if invalid.
    if (*valid)
    {
        // Create copied constraint.
        if (!name)
        {
            name = SCIPconsGetName(sourcecons);
        }
        SCIP_CALL(SCIPcreateConsOldTimeSpacing(scip,
                                                cons,
                                                name,
                                                initial,
                                                separate,
                                                enforce,
                                                check,
                                                propagate,
                                                local,
                                                modifiable,
                                                dynamic,
                                                removable,
                                                stickingatnode));

        // Mark as valid.
        *valid = TRUE;
    }

    // Done.
    return SCIP_OKAY;
}
#pragma GCC diagnostic pop

// Creates constraint handler for old time spacing constraints and include it in SCIP
SCIP_RETCODE SCIPincludeConshdlrOldTimeSpacing(
    SCIP* scip    // SCIP
)
{
    // Include constraint handler.
    SCIP_CONSHDLR* conshdlr = nullptr;
    SCIP_CALL(SCIPincludeConshdlrBasic(scip,
                                       &conshdlr,
                                       CONSHDLR_NAME,
                                       CONSHDLR_DESC,
                                       CONSHDLR_ENFOPRIORITY,
                                       CONSHDLR_CHECKPRIORITY,
                                       CONSHDLR_EAGERFREQ,
                                       CONSHDLR_NEEDSCONS,
                                       consEnfolpOldTimeSpacing,
                                       consEnfopsOldTimeSpacing,
                                       consCheckOldTimeSpacing,
                                       consLockOldTimeSpacing,
                                       nullptr));
    debug_assert(conshdlr);

    // Set callbacks.
    SCIP_CALL(SCIPsetConshdlrDelete(scip,
                                    conshdlr,
                                    consDeleteOldTimeSpacing));
    SCIP_CALL(SCIPsetConshdlrExitsol(scip,
                                     conshdlr,
                                     consExitsolOldTimeSpacing));
    SCIP_CALL(SCIPsetConshdlrCopy(scip,
                                  conshdlr,
                                  conshdlrCopyOldTimeSpacing,
                                  consCopyOldTimeSpacing));
    SCIP_CALL(SCIPsetConshdlrTrans(scip,
                                   conshdlr,
                                   consTransOldTimeSpacing));
    SCIP_CALL(SCIPsetConshdlrSepa(scip,
                                  conshdlr,
                                  consSepalpOldTimeSpacing,
                                  consSepasolOldTimeSpacing,
                                  CONSHDLR_SEPAFREQ,
                                  CONSHDLR_SEPAPRIORITY,
                                  CONSHDLR_DELAYSEPA));

    // Done.
    return SCIP_OKAY;
}

SCIP_RETCODE old_time_spacing_add_var(
    SCIP* scip,                 // SCIP
    SCIP_CONS* cons,            // old_time_spacing constraint
    SCIP_VAR* var,              // Variable
    const Time path_length,     // Path length
    const Edge* const path      // Path
)
{
    // Get constraint data.
    debug_assert(cons);
    auto consdata = reinterpret_cast<OldTimeSpacingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);

    // Check.
    debug_assert(var);
    debug_assert(SCIPconsIsTransformed(cons));
    debug_assert(SCIPvarIsTransformed(var));

    // Get necessary data
    const auto a = SCIPvardataGetAgent(SCIPvarGetData(var));
    const auto ts = SCIPprobdataGetTimeSpacing(SCIPgetProbData(scip));
    

    // Add rounding lock to the new variable.
    SCIP_CALL(SCIPlockVarCons(scip, var, cons, FALSE, TRUE));

    // Add variable to constraints. 
    for (const auto& [nta, old_time_spacing] : consdata->conflicts)
    {
        const auto& [row] = old_time_spacing;
        if (nta.a == a && nta.t < path_length && path[nta.t].n == nta.n)
        {
            SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0));
        }
        else if (nta.a != a)
        {
          for (int h = 0; h <= ts; h++)
          {
            if (nta.t + h < path_length && path[nta.t + h].n == nta.n)
            {
              SCIP_CALL(SCIPaddVarToRow(scip, row, var, 1.0 / ((double) ts + 1)));
            }
          }
        }
    }

    // Return.
    return SCIP_OKAY;
}

const HashTable<NodeTimeAgent, OldTimeSpacing>& old_time_spacing_get_constraints(
    SCIP_ProbData* probdata    // Problem data
)
{
    auto cons = SCIPprobdataGetOldTimeSpacingCons(probdata);
    debug_assert(cons);
    auto consdata = reinterpret_cast<OldTimeSpacingConsData*>(SCIPconsGetData(cons));
    debug_assert(consdata);
    return consdata->conflicts;
}

#endif
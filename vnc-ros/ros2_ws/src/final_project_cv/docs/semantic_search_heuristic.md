# Semantic Search Heuristic

This is the planned search-side model for when the target has not been directly detected yet.

## Belief Grid

The search space is discretized into cells. Each cell stores a belief score:

```text
B(g_i) = likelihood that the target is in cell g_i
```

All cells start with a uniform value.

## Updates

If the robot observes an area and sees neither the target nor a useful heuristic clue, the belief in the observed cells is reduced:

```text
B(g_i) <- B(g_i) / alpha
```

where `alpha > 1`.

If the robot detects a heuristic clue, cells inside a fixed radius around that clue are boosted:

```text
B(g_i) <- beta * B(g_i),  if distance(g_i, clue) <= r
```

where `beta > 1`.

If the target is detected, the search phase ends and control switches to direct target localization.

## Planning Use

The planner can convert belief into a navigation preference:

```text
cost(g_i) = base_cost - lambda * B(g_i)
```

Cells with stronger heuristic support become cheaper to visit, so the robot searches promising regions before neutral regions.

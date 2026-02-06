# User Rules

These rules apply to all future interactions.

1.  **Engineering & C++ Best Practices**: If I ask for something that isn't considered good engineering practice or good C++ practice (e.g., testing private methods, breaking encapsulation, unsafe memory usage), **let me know before making the changes**. Explain why it's a bad idea and propose a better alternative.

2.  **Maintain Invariants When Bounds Change**: When modifying a constraint parameter (like a max/min limit), always check if dependent values need to be re-validated. For example, if `maxStride_` is reduced, `strideLength_` must be re-clamped. Consider using a private helper method to encapsulate the update logic and enforce the invariant.

3.  **Configuration-Agnostic Code**: Prefer deriving limits and behaviors from fundamental parameters (leg dimensions, servo specs) rather than hardcoding values. This allows the code to adapt when dimensions change. Use percentages or ratios where practical (e.g., "stand at 50% of height range") so the code works across different hexapod configurations with minimal manual adjustment.

4.  **Prioritize Maintainability (DRY)**: Avoid duplicate code and duplicate values. When a value or logic is used in multiple places (e.g., init code and tests), extract it into a shared constant or function. This reduces the risk of values getting out of sync during future changes.

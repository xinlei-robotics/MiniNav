# Golden CSV baselines

End-to-end regression baselines for `sim`. CTest runs `sim` with fixed
arguments and compares its CSV output against the files here, field by field.
Any change in simulated physics, RNG consumption order, estimator math, or
planner behavior shows up as a failing `regression.golden.*` test.

## Cases

| File                       | Mode     | `sim` arguments                                                                                            |
|----------------------------|----------|------------------------------------------------------------------------------------------------------------|
| `ekf_default_seed42.csv`   | EKF      | `ekf --seed 42 --preset default`                                                                           |
| `ekf_high_noise_seed7.csv` | EKF      | `ekf --seed 7 --preset high-noise`                                                                         |
| `ekf_low_noise_seed3.csv`  | EKF      | `ekf --seed 3 --preset low-noise`                                                                          |
| `plan_office.csv`          | Planning | `plan --map maps/office.yaml --start 0.15,0.15 --goal 1.85,1.35 --config config/planner.yaml`              |
| `plan_office500.csv`       | Planning | `plan --map maps/office500.yaml --start 1.175,1.175 --goal 23.875,23.875 --config config/planner.yaml`     |

Every case also gets `--no-viz --out <file>`, and `sim` runs from the
repository root so that relative map paths, which are written into the CSV
header, do not depend on where the repository is checked out. The cases are
declared with `mininav_add_golden_test` in [`tests/CMakeLists.txt`](../CMakeLists.txt).

## Comparison rules

`csv_compare` (`tests/tools/`) compares a baseline with a fresh run:

- `#` metadata lines must match exactly, except `# generated_at`, which is
  wall-clock time.
- The column header line must match exactly.
- Fields that look like integers on both sides (encoder ticks, indices) must
  match exactly.
- Floating-point fields must satisfy `|a − b| ≤ 1e-9 · max(1, |a|)`, where
  `a` is the baseline value.
- Anything else must match as text.

The tolerance exists because trajectories are written at full `max_digits10`
precision and glibc's libm picks FMA or non-FMA implementations of `sin` /
`cos` depending on the CPU, so the last bit can differ between a development
machine and a CI runner. Real behavior changes are many orders of magnitude
larger: perturbing one noise parameter by 5 × 10⁻⁶ (relative) already fails
on the second row. On a single machine, Debug and Release output is
byte-identical.

## Running

```bash
ctest --preset test-debug -L regression --output-on-failure
```

A failing comparison prints the first differences with line number and column
name. The CSV the run produced is kept under
`build/<preset>/tests/golden_out/` for a full `diff`; CI uploads that
directory as an artifact when a job fails.

## Updating a baseline

Baselines change only on purpose. When a change is supposed to alter the
output:

```bash
cmake --build --preset build-debug --target update_golden
git diff --stat tests/golden/
```

Commit the regenerated files in the same PR as the change that caused them,
and state in the PR description why the output changed and which cases were
affected. A PR that is not meant to change behavior must not touch this
directory.

`.gitattributes` keeps these files LF in every working tree, whatever
`core.autocrlf` is set to, so they compare cleanly against program output.

<!--
Thanks for the PR. Keep the summary tight; the engineering content
belongs in the commits and the linked issue, not duplicated here.
-->

## Summary

<!-- One paragraph: what does this PR change and why? -->

## Linked issue

<!-- Use "Closes #N" to auto-close the issue on merge. -->
Closes #

## Quantitative impact

<!--
If this PR affects performance, accuracy, build time, or any
measurable property, state the delta. Otherwise write "N/A".

Examples:
- EKF RMSE on default preset, 5 seeds: odom 0.42m → ekf 0.18m (-57%)
- Configure time: 12s → 11s (no significant change)
- Binary size: +180 KB (Eigen template instantiations for EKF)
- N/A — pure refactor, no behavioral change
-->

## Checklist

- [ ] `ctest --preset test-debug` passes
- [ ] CSV regression diff is empty (or intentional change is documented)
- [ ] New public APIs have unit tests
- [ ] Docs updated if behavior or interfaces changed
- [ ] Strict warnings still pass (`-Wall -Wextra -Wconversion -Werror`)

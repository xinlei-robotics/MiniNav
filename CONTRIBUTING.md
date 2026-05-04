# Contributing to MiniNav

This document describes the development workflow for MiniNav. Even though this is currently a solo project, all contributions follow professional engineering practices.

## Development Workflow

We use a **Pull Request workflow** with squash-merging to keep the `main` branch history clean and linear.

### Branch Naming Convention

Branches follow the pattern `<type>/<short-description>`:

| Type       | Purpose                          | Example                            |
|------------|----------------------------------|------------------------------------|
| `feature/` | New functionality                | `feature/v1-ekf-prediction`        |
| `fix/`     | Bug fixes                        | `fix/eigen-const-correctness`      |
| `refactor/`| Code restructuring               | `refactor/extract-pose-module`     |
| `docs/`    | Documentation only               | `docs/architecture-diagram`        |
| `test/`    | Test-only changes                | `test/kinematics-edge-cases`       |
| `chore/`   | Build/CI/tooling                 | `chore/upgrade-gtest`              |

### Commit Message Convention

We follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>[optional scope]: <description>
[optional body]
[optional footer(s)]
```

**Common types**: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`, `perf`, `build`, `ci`

**Examples**:
- `feat(ekf): add prediction step with motion model`
- `fix(kinematics): correct yaw wrapping at ±π boundary`
- `refactor(robot): extract Pose into separate module`
- `test(planner): add edge cases for narrow corridors`

### Pull Request Process

1. **Create a feature branch** from up-to-date `main`:
```bash
   git checkout main
   git pull origin main
   git checkout -b feature/your-feature
```

2. **Develop and commit** on the branch. Messy WIP commits are fine—they will be squashed.

3. **Push and open a PR**:
```bash
   git push -u origin feature/your-feature
```
Then open the PR on GitHub. Fill out the PR template carefully.

4. **CI must pass** before merging. Fix any failures by pushing additional commits.

5. **Self-review**: open the "Files changed" tab and review your own diff as if reviewing someone else's code.

6. **Squash and merge** once CI is green and self-review is complete. The branch is auto-deleted.

## Coding Standards

### C++ Style

- **C++23 modules** for all interface files (`.ixx` for interfaces, `.cpp` for implementations)
- **Naming**: `snake_case` for variables/functions, `PascalCase` for types, `kCamelCase` for constants
- **Const-correctness**: mark methods `const` whenever possible
- **`[[nodiscard]]`** on factory functions and getters returning meaningful values
- **`noexcept`** where applicable
- **Rule of Five**: explicitly default or delete copy/move operations
- **Eigen headers** stay out of module interface export sections (use global module fragment or `.cpp`)

### Build Requirements

All PRs must compile cleanly with:
- Clang 18 / clang++-18
- `-Werror` enabled in Debug builds
- C++23 module support via `FILE_SET CXX_MODULES`

### Testing

- Use **GoogleTest** for unit tests
- Tests are discovered via `gtest_discover_tests()` and run with `ctest`
- Write tests for: kinematics, coordinate transforms, path planning edge cases, collision detection geometry

## Versioning

This project follows [Semantic Versioning](https://semver.org/):

- `MAJOR.MINOR.PATCH` (e.g. `0.2.0`)
- `MAJOR`: breaking API changes
- `MINOR`: new features (e.g. completing a milestone V1 → v0.2.0)
- `PATCH`: bug fixes within a milestone

Pre-1.0 means the API is still evolving. The project will reach `1.0.0` when it is feature-complete and recruitment-ready.

## Project Roadmap

See [README.md](README.md) for the V0–V7 milestone roadmap.
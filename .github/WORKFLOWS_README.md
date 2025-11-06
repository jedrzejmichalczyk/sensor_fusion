# GitHub Actions Workflows Documentation

This directory contains automated CI/CD workflows for the Sensor Fusion project.

## Workflows Overview

### 1. CI Build and Test (`ci.yml`) ✅

**Triggers:** Push to main/develop/claude branches, Pull Requests

**Jobs:**
- **Build and Test Matrix**
  - OS: Ubuntu 20.04, 22.04
  - Compilers: GCC, Clang
  - Build types: Debug, Release
  - Runs all unit tests with CTest
  - Uploads test results and build artifacts

- **Code Quality Checks**
  - Static analysis with `cppcheck`
  - Code formatting validation with `clang-format`
  - Uploads quality reports

- **Code Coverage**
  - Generates coverage report with `gcovr`
  - Uploads to Codecov
  - Produces HTML coverage report

**Artifacts:**
- Test results
- Build binaries (Release/GCC builds)
- Static analysis reports
- Coverage reports

---

### 2. Build Documentation (`documentation.yml`) 📚

**Triggers:** Push to main/develop, PR to main, manual dispatch

**Jobs:**
- **Build LaTeX PDF**
  - Installs full TeXLive
  - Builds `sensor_fusion_documentation.pdf`
  - Uploads PDF artifact
  - Uploads build logs on failure

- **Build Doxygen HTML**
  - Generates API documentation
  - Creates call graphs with Graphviz
  - Uploads HTML documentation

- **Deploy to GitHub Pages** (main branch only)
  - Downloads PDF and Doxygen artifacts
  - Creates index page
  - Deploys to `gh-pages` branch
  - Accessible at: `https://jedrzejmichalczyk.github.io/sensor_fusion/`

**Artifacts:**
- PDF technical documentation
- Doxygen HTML (API reference)
- LaTeX build logs (on failure)

---

### 3. Cross-Compile for Embedded (`cross-compile.yml`) 🔧

**Triggers:** Push to main/develop, Pull Requests, manual dispatch

**Jobs:**
- **ARM Cross-Compilation**
  - Targets: `armhf` (32-bit ARM), `arm64` (64-bit ARM)
  - Uses GNU ARM toolchains
  - Verifies binary architecture
  - Measures binary sizes

- **RISC-V Cross-Compilation**
  - Target: `riscv64`
  - Uses GNU RISC-V toolchain
  - Validates embedded deployment

- **Size Report**
  - Generates markdown table
  - Compares binary sizes across architectures
  - Helps track code bloat

**Artifacts:**
- ARM binaries (armhf, arm64)
- RISC-V binaries
- Size comparison report

---

### 4. Release Build (`release.yml`) 🚀

**Triggers:** Git tags (`v*.*.*`), manual dispatch

**Jobs:**
- **Create GitHub Release**
  - Generates changelog
  - Creates release draft
  - Attaches documentation

- **Build Release Assets**
  - Builds for x64, ARM (armhf, arm64)
  - Creates tarball packages
  - Uploads as release assets

**Release Package Contents:**
- `libsensor_fusion.a` (static library)
- Header files (`include/`)
- Documentation (`README.md`, `LICENSE`, PDF)

---

## Workflow Status Badges

Add these to your README.md:

```markdown
[![CI Build and Test](https://github.com/jedrzejmichalczyk/sensor_fusion/workflows/CI%20Build%20and%20Test/badge.svg)](https://github.com/jedrzejmichalczyk/sensor_fusion/actions/workflows/ci.yml)
[![Documentation](https://github.com/jedrzejmichalczyk/sensor_fusion/workflows/Build%20Documentation/badge.svg)](https://github.com/jedrzejmichalczyk/sensor_fusion/actions/workflows/documentation.yml)
[![Cross-Compile](https://github.com/jedrzejmichalczyk/sensor_fusion/workflows/Cross-Compile%20for%20Embedded/badge.svg)](https://github.com/jedrzejmichalczyk/sensor_fusion/actions/workflows/cross-compile.yml)
[![codecov](https://codecov.io/gh/jedrzejmichalczyk/sensor_fusion/branch/main/graph/badge.svg)](https://codecov.io/gh/jedrzejmichalczyk/sensor_fusion)
```

---

## Configuration Files

### `.clang-format`
Code style configuration based on LLVM with C++17 modifications:
- 4-space indentation
- 100 character line limit
- Consistent pointer alignment
- Automatic include sorting

**Usage:**
```bash
clang-format -i src/file.cpp
```

### `.codecov.yml`
Code coverage configuration:
- Target: 80% project coverage
- Threshold: ±5% variation allowed
- Ignores: tests, examples, external

### `.github/ISSUE_TEMPLATE/`
Issue templates for bug reports and feature requests

### `.github/pull_request_template.md`
PR template with checklist

---

## Local Testing

Before pushing, test locally:

### Format Check
```bash
find src include tests examples -name '*.cpp' -o -name '*.hpp' | \
  xargs clang-format --dry-run --Werror
```

### Static Analysis
```bash
cppcheck --enable=all --inconclusive --std=c++17 \
  -I include/ src/ 2>&1 | tee cppcheck.txt
```

### Build and Test
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
ctest --output-on-failure
```

### Coverage Report
```bash
mkdir build-coverage && cd build-coverage
cmake .. -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage" \
  -DCMAKE_C_FLAGS="--coverage"
make -j$(nproc)
ctest
gcovr --root .. --html coverage.html
```

---

## Troubleshooting

### Workflow Fails on Ubuntu 20.04
**Issue:** Compiler too old
**Solution:** Update CMake minimum version or compiler requirements

### LaTeX Build Fails
**Issue:** Missing packages
**Solution:** Add packages to `documentation.yml`:
```yaml
sudo apt-get install -y texlive-fonts-extra
```

### Cross-Compilation Fails
**Issue:** Missing toolchain
**Solution:** Install on local machine to test:
```bash
sudo apt-get install gcc-arm-linux-gnueabihf
```

### Coverage Report Shows 0%
**Issue:** Tests not run with coverage flags
**Solution:** Ensure `--coverage` flag is set in cmake command

---

## Adding a New Workflow

1. Create `.github/workflows/your-workflow.yml`
2. Define triggers (push, PR, schedule, etc.)
3. Set up jobs with appropriate steps
4. Test with `act` (local workflow runner):
   ```bash
   act -l  # List workflows
   act push  # Run on push event
   ```
5. Commit and push
6. Monitor in Actions tab

---

## Workflow Best Practices

1. **Use Matrix Builds** - Test multiple OS/compiler combinations
2. **Cache Dependencies** - Speed up builds with action caching
3. **Fail Fast** - Set `fail-fast: false` to see all failures
4. **Upload Artifacts** - Save build outputs for debugging
5. **Conditional Jobs** - Use `if:` to run jobs only when needed
6. **Security** - Never commit secrets, use GitHub Secrets

---

## Resources

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Workflow Syntax](https://docs.github.com/en/actions/reference/workflow-syntax-for-github-actions)
- [Workflow Commands](https://docs.github.com/en/actions/reference/workflow-commands-for-github-actions)
- [Marketplace Actions](https://github.com/marketplace?type=actions)

---

**Questions?** Open an issue or check the Actions tab for workflow runs!

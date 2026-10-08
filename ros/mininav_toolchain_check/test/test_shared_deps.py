"""Our shared libraries must not carry their own copy of spdlog or yaml-cpp.

A ROS process has already loaded the system libspdlog (through rcl_logging_spdlog). If one of our
.so files linked a second copy of a different version, its default-visibility symbols would be
bound to the copy already loaded, and the two versions' data layouts would disagree. The MiniNav
core is therefore built with MININAV_REQUIRE_SYSTEM_DEPS=ON; this test checks the result.
"""

import ctypes.util
import os
import re
import subprocess
from pathlib import Path

import pytest

LIBS = [Path(p) for p in os.environ['MININAV_CHECK_LIBS'].split(':')]


def _run(*cmd):
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def _strong_text_symbols(lib, dynamic):
    """Global (non-weak) function symbols defined in lib; template instances are weak."""
    args = ['nm', '--defined-only'] + (['-D'] if dynamic else []) + [str(lib)]
    return {line.split()[-1] for line in _run(*args).splitlines()
            if len(line.split()) == 3 and line.split()[1] == 'T'}


def _resolved_deps(lib):
    """Map of soname -> real path, as the dynamic loader resolves them for lib."""
    deps = {}
    for line in _run('ldd', str(lib)).splitlines():
        m = re.match(r'\s*(\S+) => (\S+) \(', line)
        if m:
            deps[m.group(1)] = os.path.realpath(m.group(2))
    return deps


def _find_in_ament_prefixes(relative):
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        candidate = Path(prefix) / relative
        if candidate.exists():
            return candidate
    pytest.fail(f'{relative} not found under AMENT_PREFIX_PATH')


def _system_library(name):
    soname = ctypes.util.find_library(name)
    if soname is None:
        pytest.fail(f'system library {name} not found')
    for line in _run('ldconfig', '-p').splitlines():
        if line.strip().startswith(soname + ' '):
            return Path(os.path.realpath(line.split('=>')[-1].strip()))
    pytest.fail(f'{soname} not in the ldconfig cache')


ROS_LOGGING = _find_in_ament_prefixes('lib/librcl_logging_spdlog.so')
SYSTEM_LIBS = {'spdlog': _system_library('spdlog'), 'yaml-cpp': _system_library('yaml-cpp')}


@pytest.mark.parametrize('lib', LIBS, ids=lambda p: p.name)
@pytest.mark.parametrize('dep', sorted(SYSTEM_LIBS))
def test_no_static_copy(lib, dep):
    system_symbols = _strong_text_symbols(SYSTEM_LIBS[dep], dynamic=True)
    assert system_symbols, f'no exported functions found in {SYSTEM_LIBS[dep]}'
    overlap = _strong_text_symbols(lib, dynamic=False) & system_symbols
    assert not overlap, f'{lib.name} defines {len(overlap)} {dep} functions, e.g. {min(overlap)}'


@pytest.mark.parametrize('lib', LIBS, ids=lambda p: p.name)
def test_spdlog_is_the_one_ros_logging_loads(lib):
    def spdlog_of(deps):
        return {k: v for k, v in deps.items() if re.match(r'libspdlog\.so', k)}

    ros_spdlog = spdlog_of(_resolved_deps(ROS_LOGGING))
    ours = spdlog_of(_resolved_deps(lib))
    assert ros_spdlog, 'rcl_logging_spdlog does not link libspdlog'
    for soname, path in ours.items():
        assert ros_spdlog.get(soname) == path, f'{lib.name} loads {path}, ROS logging {ros_spdlog}'

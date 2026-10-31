"""5D generative-design tests: experimental, out of the default run.

Every test collected under tests/gen/ gets the ``experimental``
marker (Stage 5D work-in-progress). The default suite deselects it
(see pyproject ``addopts``). To run these explicitly::

    pytest tests/gen -m experimental -o addopts=''
"""

import pathlib

import pytest

HERE = pathlib.Path(__file__).resolve().parent


def pytest_collection_modifyitems(items):
    # NOTE: collection hooks run session-wide once this conftest is
    # loaded, so filter by path instead of marking everything.
    for item in items:
        try:
            path = pathlib.Path(str(item.fspath)).resolve()
        except (OSError, ValueError):
            continue
        if HERE in path.parents or path == HERE:
            item.add_marker(pytest.mark.experimental)

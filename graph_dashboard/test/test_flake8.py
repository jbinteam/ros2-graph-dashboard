# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # Package-local config: ament defaults plus the repo's double-quote style
    # and the tests' env-before-import allowance (rationale in flake8.ini).
    config = str(Path(__file__).parents[1] / "flake8.ini")
    rc, errors = main_with_errors(argv=["--config", config])
    assert rc == 0, f"Found {len(errors)} code style errors / warnings:\n" + "\n".join(errors)

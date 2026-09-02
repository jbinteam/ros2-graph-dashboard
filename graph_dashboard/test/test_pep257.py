# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from ament_pep257.main import main
import pytest

# Ignore set mirrors ament_flake8's bundled config (missing-docstring codes
# D1xx, D203, D404) plus D213: this repo writes multi-line docstring
# summaries on the FIRST line (D212 style, what ament_flake8 also expects),
# and D213 is the mutually-exclusive opposite of that choice.
_IGNORES = [
    "D100", "D101", "D102", "D103", "D104", "D105", "D106", "D107",
    "D203", "D213", "D404",
]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[".", "test", "--ignore", *_IGNORES])
    assert rc == 0, "Found docstring style errors / warnings"

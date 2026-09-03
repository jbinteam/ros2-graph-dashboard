# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from setuptools import find_packages, setup

package_name = "graph_dashboard"

setup(
    name=package_name,
    version="0.7.0",
    packages=find_packages(exclude=["test"]),
    # Web assets (index.html + vendored vis-network) ship inside the Python
    # package so server.py can locate them via __file__ regardless of how
    # the package was installed.
    package_data={package_name: ["web/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="JB",
    maintainer_email="i.jiraphan@gmail.com",
    description=(
        "Static ROS-graph dashboard: AST scanner for every declared "
        "node/topic in the workspace plus a local rqt_graph-style web view."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan = graph_dashboard.scanner:main",
            "serve = graph_dashboard.server:main",
            "bench_test = graph_dashboard.bench_test:main",
        ],
    },
)

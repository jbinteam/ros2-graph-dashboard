# graph_dashboard

Local web dashboard showing the **whole project's declared ROS graph** —
every node, topic, and pub/sub edge found by statically scanning the Python
sources under `ros_ws/src/` — with a live overlay painted on top.

`rqt_graph` shows only what is running *right now*. This dashboard shows the
architecture as the source code declares it, whether or not anything is
running, and then marks what actually *is* running. That is the view a team
needs when work is split across packages: the full picture, who publishes
what, and what your change might ripple into.

**Scanned, never executed:** the scanner parses source text with Python's
`ast` module. It never imports or runs any scanned package — the frozen
`vla_*` sources (OpenVLA hold) are included in the picture without a single
line of them executing.

## Quickstart

```bash
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash

ros2 run graph_dashboard scan          # writes ./graph.json, prints a summary
ros2 run graph_dashboard serve         # dashboard on http://127.0.0.1:8091/
ros2 run graph_dashboard bench_test    # end-to-end self-test (ephemeral port)
```

All three auto-detect the workspace `src/` directory (any workspace name)
by walking up from the current directory until they find a `src/`
containing at least one `package.xml` — run them from the workspace root,
from inside `src/`, or from a package directory. Pass
`--src /path/to/ws/src` to point elsewhere. `serve`
also takes `--host` (default 127.0.0.1) and `--port` (default **8091** —
8090 belongs to the frozen `webui/`). `scan` takes `--output` (default
`./graph.json`).

## Using the page

- **Hover** a node or topic: lights its full transitive chain (everything
  that can affect it or be affected by it, cycles included), dims the rest.
- **Click** a node or topic: splits the page; the bottom **focus panel**
  shows the direct neighborhood — for a node, its topics (±1) and the
  before/after nodes (±2); for a topic, its publishers and subscribers.
  Click inside the panel to walk the chain hop by hop; `Esc` or ✕ closes.
  Focusing a **topic** also opens a live tap strip (measured Hz, bandwidth,
  latest message as thumbnail or field tree) — the server subscribes to
  that one topic only while the panel is open and releases it ~5 s after
  the panel closes. The strip's Hz/bandwidth are the topic's TRUE measured
  rate; the display itself is polled at only 2 Hz and shows the newest
  message each poll, so a choppy preview does not mean a slow topic.
- **Legend chips** are filters: click a package chip to hide/show that
  package (topics hide only when *every* node touching them is hidden);
  "hide test harnesses" hides the dashed bench/test elements. Filters apply
  to the main graph only — the focus panel always shows the true
  neighborhood, even filtered-out elements, so the ego view never lies.
- **Tooltips** carry the details: nodes list their declared pubs/subs with
  message types and best-effort static QoS; topics list publishers and
  subscribers. QoS shows what the scanner can prove from the call site
  (literal depths, reliability/durability policies, the repo's
  `_*_qos()` helper pattern); anything else reads "unknown" — never a guess.

### URL parameters (shareable per-team deep links)

| Param | Effect | Example |
|-------|--------|---------|
| `?highlight=<name>` | light that element's chain on load | `/?highlight=hand_tracking` |
| `?focus=<name>` | open the focus panel on load | `/?focus=/vla/estop` |
| `?hide=<list>` | comma-separated packages and/or `tests` to hide | `/?hide=vla_sim,tests` |

Names accept a node name, a topic name (with or without the leading `/`),
or a full element id (`node:<pkg>/<name>`, `topic:/<name>`). Parameters
compose: `/?focus=hand_tracking&hide=vla_control,tests`.

## API

| Endpoint | Returns |
|----------|---------|
| `GET /api/graph` | full static graph JSON (nodes, topics, edges with QoS, per-element reachability closures, coverage summary) — re-scanned per request, so a reload always reflects the current source |
| `GET /api/ego?id=<element>` | signed-distance ego graph of one element (negative = inputs, positive = outputs), clamped to the direct neighborhood (±2 for nodes, ±1 for topics); 404 for unknown ids |
| `GET /api/live` | live-graph snapshot: running nodes and topic endpoint counts, sampled every ~2 s |
| `GET /api/tap?id=topic:/<name>` | on-demand live tap of ONE topic: Hz, bandwidth, latest message (JPEG thumbnail for images, truncated field tree otherwise). Poll-driven lifecycle — first poll subscribes (best-effort, depth 1, raw), ~5 s without polls unsubscribes; max 3 concurrent taps; honest JSON statuses for dead topics; 404 for unknown ids |
| `GET /` | the dashboard page |
| `GET /vendor/vis-network.min.js` | vendored render library (offline lab use) |

## Design rules

- **Topic-name resolution** (static, in order): string literal at the call
  site → simple local/self/module-constant assignment →
  `declare_parameter("x_topic", default)` **default** via either rclpy
  accessor spelling. Unresolvable names appear as `?<expr>` placeholders
  and are counted in the coverage summary. The static view shows declared
  defaults; runtime remaps/params-YAML overrides are not applied.
- **Node naming:** the literal in `super().__init__("name")`, else the
  class name, else the file stem for module-level pub/sub calls.
- **C++ (rclcpp) coverage is heuristic**, not a real C++ parse: it finds
  `create_publisher<T>("topic", qos)` / `create_subscription<T>(...)` call
  sites in `*.cpp/*.cc/*.hpp/*.hh`, takes the topic from the string
  literal (a variable becomes the same `?<expr>` placeholder), the message
  type from the template argument, the node name from a `Node("name")`
  constructor literal (else the subclass name, else the file stem — one
  node per file), and QoS only from literal spellings (integer depths,
  `rclcpp::QoS(N)`, `KeepLast(N)`, `SensorDataQoS`, `SystemDefaultsQoS`,
  `.best_effort()`-style modifiers). **Known limits:** composed /
  pluginlib-loaded nodes, launch-time remappings, and macro-built names
  are not resolved — they show as placeholders or are missed in the
  static view. Anything actually running still appears via the live
  overlay, so the picture degrades to "live-only", never to invisible.
  C++ nodes carry a dark ring and a "C++ (heuristic scan)" tooltip tag.
- **Layout:** left-to-right longest-path layering over the acyclic part of
  the graph. The graph genuinely contains cycles (test harnesses subscribe
  downstream and publish upstream), so back edges are found by DFS first
  and excluded from layering — they simply draw right-to-left, as
  rqt_graph does. Column order minimizes crossings (barycenter pass).
- **Hover chain:** true transitive closure over the real directed graph,
  cycles included — harness feedback paths deliberately light up.
- **Ego placement:** BFS distance from the focused element, clamped as
  above. An element reachable both upstream and downstream appears exactly
  once: the side with the smaller |distance| wins, ties go upstream (the
  tooltip says so when it happens).

## Live overlay

A background thread in `serve` runs a read-only rclpy probe node (samples
node names and topic endpoints every ~2 s; publishes nothing). A static
node is marked **running** when its declared node name equals a live
node's base name — namespaces are ignored, since this repo declares none.
Running nodes render saturated with a bold green border; declared-but-idle
nodes dim; live nodes that appear in no source file (rviz, ros2cli
daemons, …) render as dotted ellipses beneath the graph so the picture
never hides a running process.

**Graceful degradation:** rclpy is imported lazily inside the probe
thread. Where ROS isn't sourced, `/api/live` reports
`{"available": false, "reason": ...}`, the page shows a "live:
unavailable" chip, and everything static keeps working.

## Compatibility

- **ROS 2 only.** ROS 1 / rospy is unsupported (different graph model and
  client APIs; the scanner would find nothing and the probe cannot join a
  ROS 1 graph).
- **Tested on Jazzy; expected to work on Humble and newer** — the server
  is pure stdlib, the probe/tap use long-stable rclpy APIs
  (`get_node_names_and_namespaces`, `create_subscription(..., raw=True)`),
  and the installer auto-sources whatever `/opt/ros/<distro>` it finds.
- **Python packages get full static coverage** (AST scan, parameter-default
  resolution, QoS extraction). **C++ packages get heuristic coverage** —
  see the design rules above for exactly what is and isn't resolved.
- **The live overlay and topic taps are language-agnostic**: they observe
  the DDS graph itself, so C++, Python, and composed nodes all appear when
  running regardless of what the static scan could see.

## Installing into another workspace

Generate a self-contained installer and hand the single file to a teammate:

```bash
bash scripts/make_install.sh      # from the repo root — writes ./install.sh
```

`install.sh` carries the whole package as an embedded tarball (no git, no
network, no root). Dropped into ANY directory and run with
`bash install.sh`, it creates or reuses `./src`, checks the environment
(sourcing the newest `/opt/ros/*/setup.bash` if none is sourced, with
actionable errors otherwise), extracts, builds with colcon, and runs the
package self-test on an ephemeral port. Re-running refuses to overwrite an
existing `src/graph_dashboard` unless given `--force`, which keeps a
timestamped backup at the workspace root. In a workspace without this
repo's packages, `bench_test` automatically skips the repo-contract checks
and runs its generic subset.

## Vendored third-party code

`graph_dashboard/web/vis-network.min.js` — vis-network 9.1.9
(<https://visjs.github.io/vis-network/>), vendored for offline lab use.
Dual-licensed Apache-2.0 / MIT; the license banner is retained at the top
of the vendored file.

## Tests

`colcon test --packages-select graph_dashboard` runs the ament lint suite
(copyright, flake8, pep257, xmllint) plus unit tests for the scanner's
resolution tiers, reachability closures (cycle-safe), and ego depth
clamping. `ros2 run graph_dashboard bench_test` is the end-to-end check:
contract edges in a fresh scan, every endpoint of a real server boot on an
ephemeral port, the ego clamp, the 404 path, the vendored asset, and clean
shutdown.

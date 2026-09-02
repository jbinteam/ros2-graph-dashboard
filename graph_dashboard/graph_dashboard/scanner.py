# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""scan — static ROS-graph extractor over every Python source in the workspace.

Unlike rqt_graph (which introspects the LIVE graph), this reads source code
with the `ast` module — never importing or executing any scanned file — and
reports every `create_publisher` / `create_subscription` call declared
anywhere under `ros_ws/src/`, whether or not those nodes are running.

Topic-name resolution is best-effort static analysis, in this order:
  1. a string literal at the call site;
  2. a local/self variable assigned a string literal, followed through
     simple assignments (module constants included);
  3. the repo's dominant pattern:
         self.declare_parameter("x_topic", "/default")
         x = self.get_parameter("x_topic").value
     which resolves to the declared DEFAULT (a launch file or params YAML
     can override it at runtime — the static view shows the declared
     wiring, not a live remap).
Anything else (f-strings, arithmetic, values passed in from outside the
class) is recorded as a dynamic placeholder "?<expr>" and counted in the
coverage summary instead of crashing or being silently dropped.

Node naming: the string literal in `super().__init__("name")` within the
same class; falls back to the class name, or the file stem for calls made
outside any class (test harnesses, bench scripts).
"""
import argparse
import ast
import datetime
import json
from pathlib import Path
import re
import sys

_EXCLUDE_DIRS = {"build", "install", "log", "__pycache__", ".pytest_cache", ".git"}
_PUBSUB = {"create_publisher", "create_subscription"}
_CPP_EXTS = {".cpp", ".cc", ".hpp", ".hh"}


def _src_has_packages(src_dir: Path):
    """Report whether a colcon package (package.xml) lives beneath src_dir.

    Bounded glob depth (packages may sit under a repo subdirectory), so
    walking up through huge trees stays cheap.
    """
    for pattern in ("package.xml", "*/package.xml", "*/*/package.xml",
                    "*/*/*/package.xml"):
        try:
            if next(iter(src_dir.glob(pattern)), None) is not None:
                return True
        except OSError:
            pass
    return False


def _find_src_root(start: Path):
    """Locate the workspace src/ directory, name-agnostic.

    Walking up from `start`: a dir literally named src containing at least
    one package.xml wins (covers running from inside src or a package);
    else a src/ child with packages (any workspace name — my_ros_ws, ws,
    …); else the legacy ros_ws/src spelling (so running from this repo's
    root, one level above the workspace, keeps working).
    """
    for d in (start, *start.parents):
        if d.name == "src" and _src_has_packages(d):
            return d
        cand = d / "src"
        if cand.is_dir() and _src_has_packages(cand):
            return cand
        legacy = d / "ros_ws" / "src"
        if legacy.is_dir() and _src_has_packages(legacy):
            return legacy
    return None


def _iter_py_files(src_root: Path):
    for path in sorted(src_root.rglob("*.py")):
        if any(part in _EXCLUDE_DIRS for part in path.parts):
            continue
        yield path


def _iter_cpp_files(src_root: Path):
    for path in sorted(src_root.rglob("*")):
        if path.suffix not in _CPP_EXTS:
            continue
        if any(part in _EXCLUDE_DIRS for part in path.parts):
            continue
        yield path


def _const_str(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _get_parameter_key(node):
    """Return "key" from either rclpy parameter-read accessor spelling.

    Matches `<x>.get_parameter("key").value` and
    `<x>.get_parameter("key").get_parameter_value().string_value`.
    """
    if not isinstance(node, ast.Attribute):
        return None
    if node.attr == "value":
        call = node.value
    elif node.attr == "string_value":
        inner = node.value
        if not (
            isinstance(inner, ast.Call)
            and isinstance(inner.func, ast.Attribute)
            and inner.func.attr == "get_parameter_value"
        ):
            return None
        call = inner.func.value
    else:
        return None
    if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)):
        return None
    if call.func.attr != "get_parameter" or not call.args:
        return None
    return _const_str(call.args[0])


def _parse_qos_profile_call(call, fn_params=None):
    """Parse a literal QoSProfile(...) call; None if `call` is not one.

    Extracts what is statically certain: integer depth literals, and
    reliability/durability policy attribute names. A depth passed through a
    helper's parameter is marked "param" for the caller to fill from its
    own literal argument. Everything else stays None ("unknown"), never a
    guess.
    """
    if not (
        isinstance(call, ast.Call)
        and (
            (isinstance(call.func, ast.Name) and call.func.id == "QoSProfile")
            or (isinstance(call.func, ast.Attribute) and call.func.attr == "QoSProfile")
        )
    ):
        return None
    info = {"depth": None, "reliability": None, "durability": None}
    for kw in call.keywords:
        if kw.arg == "depth":
            if isinstance(kw.value, ast.Constant) and isinstance(kw.value.value, int):
                info["depth"] = kw.value.value
            elif isinstance(kw.value, ast.Name) and fn_params and kw.value.id in fn_params:
                info["depth"] = "param"
        elif kw.arg in ("reliability", "durability") and isinstance(kw.value, ast.Attribute):
            info[kw.arg] = kw.value.attr.lower()
    return info


def _collect_qos_helpers(tree):
    """Map module-level helper-function name -> QoS info it returns.

    Covers the repo's `_sensor_data_qos(depth)` / `_reliable_qos(1)` /
    `_latched_qos(10)` pattern: a function whose return statement is a
    literal QoSProfile(...) call.
    """
    helpers = {}
    for fn in tree.body:
        if not isinstance(fn, ast.FunctionDef):
            continue
        params = {a.arg for a in fn.args.args}
        for stmt in ast.walk(fn):
            if isinstance(stmt, ast.Return) and stmt.value is not None:
                info = _parse_qos_profile_call(stmt.value, fn_params=params)
                if info:
                    helpers[fn.name] = info
                    break
    return helpers


_QOS_UNKNOWN = {"depth": None, "reliability": None, "durability": None}


def _qos_at_call(expr, qos_helpers, raw_assigns):
    """Best-effort static QoS of one pub/sub call site (never a guess)."""
    if expr is None:
        return dict(_QOS_UNKNOWN)
    # Bare int is rclpy shorthand for QoSProfile(depth=n), whose documented
    # defaults are reliable/volatile — stated, not guessed.
    if isinstance(expr, ast.Constant) and isinstance(expr.value, int):
        return {"depth": expr.value, "reliability": "reliable", "durability": "volatile"}
    if isinstance(expr, ast.Name) and expr.id in raw_assigns:
        expr = raw_assigns[expr.id]
    if isinstance(expr, ast.Call):
        direct = _parse_qos_profile_call(expr)
        if direct:
            return direct
        if isinstance(expr.func, ast.Name):
            fname = expr.func.id
        elif isinstance(expr.func, ast.Attribute):
            fname = expr.func.attr
        else:
            fname = None
        if fname in qos_helpers:
            info = dict(qos_helpers[fname])
            if info["depth"] == "param":
                info["depth"] = next(
                    (a.value for a in expr.args
                     if isinstance(a, ast.Constant) and isinstance(a.value, int)),
                    None,
                )
            return info
    return dict(_QOS_UNKNOWN)


def _walk_skip_nested_classes(node):
    """Yield descendants of `node` without descending into nested ClassDefs."""
    stack = list(ast.iter_child_nodes(node))
    while stack:
        child = stack.pop()
        yield child
        if not isinstance(child, ast.ClassDef):
            stack.extend(ast.iter_child_nodes(child))


class _Scope:
    """Best-effort constant environment for one class (or the module level)."""

    def __init__(self, module_consts):
        self.module_consts = module_consts
        self.param_defaults = {}  # declare_parameter name -> str default
        self.assigns = {}  # "x" or "self.x" -> str value
        self.raw = {}  # "x" or "self.x" -> raw value AST (for QoS lookup)

    def collect(self, body_nodes):
        # Pass 1: parameter defaults (they may be declared after first use
        # in source order only pathologically, but two passes cost nothing).
        for n in body_nodes:
            if (
                isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute)
                and n.func.attr == "declare_parameter"
                and len(n.args) >= 2
            ):
                key = _const_str(n.args[0])
                val = _const_str(n.args[1])
                if key is not None and val is not None:
                    self.param_defaults[key] = val

    def collect_assigns(self, body_nodes):
        for n in body_nodes:
            if not (isinstance(n, ast.Assign) and len(n.targets) == 1):
                continue
            target = n.targets[0]
            if isinstance(target, ast.Name):
                name = target.id
            elif (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                name = "self." + target.attr
            else:
                continue
            self.raw[name] = n.value
            val = self.resolve(n.value)
            if val is not None:
                self.assigns[name] = val

    def resolve(self, expr):
        """Resolve `expr` to a string, or None if it is not statically known."""
        lit = _const_str(expr)
        if lit is not None:
            return lit
        key = _get_parameter_key(expr)
        if key is not None:
            return self.param_defaults.get(key)
        if isinstance(expr, ast.Name):
            return self.assigns.get(expr.id, self.module_consts.get(expr.id))
        if (
            isinstance(expr, ast.Attribute)
            and isinstance(expr.value, ast.Name)
            and expr.value.id == "self"
        ):
            return self.assigns.get("self." + expr.attr)
        return None


def _msg_type_name(expr, msg_imports):
    """Render the message-type argument, expanding `from x.msg import Y` imports."""
    if isinstance(expr, ast.Name) and expr.id in msg_imports:
        return msg_imports[expr.id]
    try:
        return ast.unparse(expr)
    except Exception:  # unparse failure must never kill the scan
        return "?"


def _node_name_for_class(cls):
    """Extract the literal from super().__init__("name"), else the class name."""
    for n in _walk_skip_nested_classes(cls):
        if (
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == "__init__"
            and isinstance(n.func.value, ast.Call)
            and isinstance(n.func.value.func, ast.Name)
            and n.func.value.func.id == "super"
            and n.args
        ):
            name = _const_str(n.args[0])
            if name is not None:
                return name
    return cls.name


class GraphBuilder:
    """Accumulates nodes / topics / edges across all scanned files."""

    def __init__(self):
        self.nodes = {}  # id -> record
        self.topics = {}  # name -> record
        self.edges = []
        self._edge_keys = set()
        self.dynamic_sites = []  # coverage: unresolved call sites
        self.parse_errors = []
        self.files_scanned = 0
        self.cpp_files_scanned = 0

    def add_call(self, call, scope, node_id, msg_imports, rel_file, qos_helpers):
        kind = "pub" if call.func.attr == "create_publisher" else "sub"
        if len(call.args) < 2:
            return
        msg_type = _msg_type_name(call.args[0], msg_imports)
        qos_index = 2 if kind == "pub" else 3
        qos_expr = call.args[qos_index] if len(call.args) > qos_index else None
        qos = _qos_at_call(qos_expr, qos_helpers, scope.raw)
        topic_expr = call.args[1]
        topic = scope.resolve(topic_expr)
        if topic is None:
            try:
                expr_text = ast.unparse(topic_expr)
            except Exception:
                expr_text = "<unparseable>"
        else:
            expr_text = None
        self.record(kind, msg_type, topic, expr_text, qos, node_id, rel_file, call.lineno)

    def record(self, kind, msg_type, topic, expr_text, qos, node_id, rel_file, line):
        """Shared edge/topic recorder for the Python and C++ scanners.

        `topic` is the resolved name, or None with `expr_text` describing
        the unresolvable expression (becomes a "?<expr>" placeholder).
        """
        dynamic = topic is None
        if dynamic:
            topic = "?" + expr_text
            self.dynamic_sites.append(
                {"file": rel_file, "line": line, "expr": expr_text, "kind": kind}
            )
        trec = self.topics.setdefault(
            topic, {"name": topic, "msg_types": [], "dynamic": dynamic}
        )
        if msg_type not in trec["msg_types"]:
            trec["msg_types"].append(msg_type)
        if kind == "pub":
            source, target = node_id, "topic:" + topic
        else:
            source, target = "topic:" + topic, node_id
        key = (source, target, kind)
        if key not in self._edge_keys:
            self._edge_keys.add(key)
            self.edges.append(
                {"source": source, "target": target, "kind": kind,
                 "msg_type": msg_type, "qos": qos}
            )

    def ensure_node(self, node_id, node_name, package, rel_file, class_name, is_test,
                    language="python"):
        self.nodes.setdefault(
            node_id,
            {
                "id": node_id,
                "node_name": node_name,
                "package": package,
                "source_file": rel_file,
                "class_name": class_name,
                "is_test": is_test,
                "language": language,
            },
        )


# --------------------------------------------------------------------------
# C++ (rclcpp) heuristic scanner — regex-based, not a real C++ parse.
# Covers the dominant idioms: create_publisher<T>("topic", qos) /
# create_subscription<T>("topic", qos, cb), Node("name") constructor calls,
# and literal QoS spellings. Composed/pluginlib nodes, remappings, and
# macro-built names are NOT resolved (README states this); a variable topic
# becomes the same "?<expr>" placeholder the Python scanner uses.

_CPP_CALL_RE = re.compile(
    r"create_(publisher|subscription)\s*<\s*([A-Za-z0-9_:\s]+?)\s*>\s*\("
)
_CPP_NODE_NAME_RE = re.compile(r'\bNode\s*\(\s*"([^"]+)"')
_CPP_CLASS_RE = re.compile(
    r"class\s+(\w+)\s*(?:final\s*)?:\s*public\s+(?:rclcpp::)?Node\b"
)


def _cpp_strip_comments(text):
    """Remove comments, preserving line numbers for reported call sites."""
    text = re.sub(
        r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S
    )
    return re.sub(r"//[^\n]*", "", text)


def _cpp_call_args(text, start, max_args=4, max_span=2000):
    """Split the top-level arguments of a call whose '(' is at start-1."""
    args = []
    cur = []
    depth = 1
    in_str = False
    i = start
    end = min(len(text), start + max_span)
    while i < end and depth > 0 and len(args) < max_args:
        ch = text[i]
        if in_str:
            cur.append(ch)
            if ch == '"' and text[i - 1] != "\\":
                in_str = False
        elif ch == '"':
            in_str = True
            cur.append(ch)
        elif ch in "([{":
            depth += 1
            cur.append(ch)
        elif ch in ")]}":
            depth -= 1
            if depth > 0:
                cur.append(ch)
        elif ch == "," and depth == 1:
            args.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
        i += 1
    if cur and len(args) < max_args:
        args.append("".join(cur).strip())
    return args


def _cpp_qos(arg):
    """Best-effort QoS from an rclcpp argument string; unknown stays None."""
    info = dict(_QOS_UNKNOWN)
    if arg is None:
        return info
    a = arg.strip()
    if re.fullmatch(r"\d+", a):
        # Bare int = KeepLast(n) with rclcpp defaults (documented, not guessed).
        return {"depth": int(a), "reliability": "reliable", "durability": "volatile"}
    m = re.search(r"\bQoS\s*[({]\s*(\d+)\s*[)}]", a) or re.search(
        r"\bKeepLast\s*\(\s*(\d+)\s*\)", a
    )
    if m:
        info["depth"] = int(m.group(1))
    if "SensorDataQoS" in a:
        info["reliability"] = "best_effort"
        info["durability"] = "volatile"
        if info["depth"] is None:
            info["depth"] = 5  # rmw_qos_profile_sensor_data's documented depth
    if "SystemDefaultsQoS" in a:
        info["reliability"] = "system_default"
        info["durability"] = "system_default"
    if re.search(r"\bbest_effort\s*\(", a):
        info["reliability"] = "best_effort"
    if re.search(r"\breliable\s*\(", a):
        info["reliability"] = "reliable"
    if re.search(r"\btransient_local\s*\(", a):
        info["durability"] = "transient_local"
    return info


def _scan_cpp_file(path, src_root, builder):
    rel_file = str(path.relative_to(src_root))
    package = path.relative_to(src_root).parts[0]
    is_test = "test" in path.relative_to(src_root).parts[1:-1]
    try:
        text = _cpp_strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    except OSError as exc:
        builder.parse_errors.append({"file": rel_file, "error": str(exc)})
        return
    builder.files_scanned += 1
    builder.cpp_files_scanned += 1

    calls = list(_CPP_CALL_RE.finditer(text))
    if not calls:
        return
    # Whole-file node attribution (heuristic): the first Node("name")
    # constructor literal, else the rclcpp::Node subclass name, else stem.
    name_m = _CPP_NODE_NAME_RE.search(text)
    class_m = _CPP_CLASS_RE.search(text)
    node_name = (
        name_m.group(1) if name_m else (class_m.group(1) if class_m else path.stem)
    )
    node_id = "node:{}/{}".format(package, node_name)
    builder.ensure_node(
        node_id, node_name, package, rel_file,
        class_m.group(1) if class_m else None, is_test, language="cpp",
    )
    for m in calls:
        kind = "pub" if m.group(1) == "publisher" else "sub"
        msg_type = re.sub(r"\s+", "", m.group(2)).replace("::", "/")
        args = _cpp_call_args(text, m.end())
        if not args:
            continue
        topic_arg = args[0]
        lit = re.fullmatch(r'"([^"]*)"', topic_arg)
        topic = lit.group(1) if lit else None
        expr_text = None if lit else re.sub(r"\s+", " ", topic_arg)
        qos = _cpp_qos(args[1] if len(args) > 1 else None)
        line = text.count("\n", 0, m.start()) + 1
        builder.record(kind, msg_type, topic, expr_text, qos, node_id, rel_file, line)


def _scan_file(path, src_root, builder):
    rel_file = str(path.relative_to(src_root))
    package = path.relative_to(src_root).parts[0]
    is_test = "test" in path.relative_to(src_root).parts[1:-1]
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (SyntaxError, UnicodeDecodeError) as exc:
        builder.parse_errors.append({"file": rel_file, "error": str(exc)})
        return
    builder.files_scanned += 1

    qos_helpers = _collect_qos_helpers(tree)
    msg_imports = {}
    module_consts = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom) and n.module and n.module.endswith(".msg"):
            for alias in n.names:
                msg_imports[alias.asname or alias.name] = (
                    n.module.replace(".", "/") + "/" + alias.name
                )
    module_raw = {}
    for stmt in tree.body:
        if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1:
            target = stmt.targets[0]
            if isinstance(target, ast.Name):
                module_raw[target.id] = stmt.value
                val = _const_str(stmt.value)
                if val is not None:
                    module_consts[target.id] = val

    classes = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    for cls in classes:
        body = list(_walk_skip_nested_classes(cls))
        calls = [
            n
            for n in body
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr in _PUBSUB
        ]
        if not calls:
            continue
        scope = _Scope(module_consts)
        scope.raw.update(module_raw)  # class assigns override below
        scope.collect(body)
        scope.collect_assigns(body)
        node_name = _node_name_for_class(cls)
        node_id = "node:{}/{}".format(package, node_name)
        builder.ensure_node(node_id, node_name, package, rel_file, cls.name, is_test)
        for call in calls:
            builder.add_call(call, scope, node_id, msg_imports, rel_file, qos_helpers)

    # Calls made outside any class (bench scripts, test drivers on helper
    # nodes) are attributed to a file-stem pseudo-node so they still appear.
    module_body = list(_walk_skip_nested_classes(tree))
    module_calls = [
        n
        for n in module_body
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr in _PUBSUB
    ]
    if module_calls:
        scope = _Scope(module_consts)
        scope.raw.update(module_raw)
        scope.collect(module_body)
        scope.collect_assigns(module_body)
        node_name = path.stem
        node_id = "node:{}/{}".format(package, node_name)
        builder.ensure_node(node_id, node_name, package, rel_file, None, is_test)
        for call in module_calls:
            builder.add_call(call, scope, node_id, msg_imports, rel_file, qos_helpers)


def compute_reachability(edges):
    """Transitive up/down closure for every element that touches an edge.

    Returns {id: {"up": [ancestor ids], "down": [descendant ids]}} following
    the REAL directed graph (cycles included, BFS with a visited set, so a
    test harness publishing upstream cannot loop it). The element itself is
    not listed in its own closures even when it sits on a cycle.
    """
    fwd = {}
    rev = {}
    for e in edges:
        fwd.setdefault(e["source"], []).append(e["target"])
        rev.setdefault(e["target"], []).append(e["source"])

    def _bfs(start, adj):
        seen = set()
        queue = [start]
        while queue:
            for nxt in adj.get(queue.pop(), []):
                if nxt != start and nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return sorted(seen)

    return {
        i: {"up": _bfs(i, rev), "down": _bfs(i, fwd)}
        for i in set(fwd) | set(rev)
    }


def compute_ego(edges, center, max_depth=None):
    """Signed BFS distance from `center` for its ego-graph.

    Negative = upstream (inputs), positive = downstream (outputs), 0 = the
    center itself. `max_depth` clamps the BFS radius per side (None = whole
    transitive closure): the focus panel uses 2 for a node center (its
    topics at ±1 plus the "before"/"after" nodes at ±2) and 1 for a topic
    center (its publishers/subscribers). An element reachable in BOTH
    directions (harness cycles) appears exactly once, deterministically:
    the side with the smaller absolute distance wins, ties go upstream.
    Returns {"center", "levels": {id: signed int}, "dual": [ids that were
    reachable both ways]}.
    """
    fwd = {}
    rev = {}
    for e in edges:
        fwd.setdefault(e["source"], []).append(e["target"])
        rev.setdefault(e["target"], []).append(e["source"])

    def _dists(adj):
        dist = {}
        frontier = [center]
        d = 0
        while frontier:
            if max_depth is not None and d >= max_depth:
                break
            d += 1
            nxt = []
            for u in frontier:
                for v in adj.get(u, []):
                    if v != center and v not in dist:
                        dist[v] = d
                        nxt.append(v)
            frontier = nxt
        return dist

    up = _dists(rev)
    down = _dists(fwd)
    levels = {center: 0}
    dual = []
    for elem in set(up) | set(down):
        if elem in up and elem in down:
            dual.append(elem)
            levels[elem] = -up[elem] if up[elem] <= down[elem] else down[elem]
        elif elem in up:
            levels[elem] = -up[elem]
        else:
            levels[elem] = down[elem]
    return {"center": center, "levels": levels, "dual": sorted(dual)}


def scan_workspace(src_root):
    """Scan every Python file under `src_root`; return the graph as a dict."""
    src_root = Path(src_root).resolve()
    builder = GraphBuilder()
    for path in _iter_py_files(src_root):
        _scan_file(path, src_root, builder)
    for path in _iter_cpp_files(src_root):
        _scan_cpp_file(path, src_root, builder)

    topics = []
    for name, trec in sorted(builder.topics.items()):
        topics.append(
            {
                "name": name,
                "msg_type": " | ".join(trec["msg_types"]),
                "dynamic": trec["dynamic"],
            }
        )
    summary = {
        "files_scanned": builder.files_scanned,
        "cpp_files_scanned": builder.cpp_files_scanned,
        "parse_errors": builder.parse_errors,
        "node_count": len(builder.nodes),
        "topic_count": len(topics),
        "edge_count": len(builder.edges),
        "dynamic_topic_count": sum(1 for t in topics if t["dynamic"]),
        "dynamic_call_sites": builder.dynamic_sites,
    }
    return {
        "generated": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "src_root": str(src_root),
        "nodes": sorted(builder.nodes.values(), key=lambda n: n["id"]),
        "topics": topics,
        "edges": builder.edges,
        # Per-element transitive up/down reachability, precomputed here so
        # the web view's hover-chain highlight is trivial (and testable).
        "closures": compute_reachability(builder.edges),
        "summary": summary,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Statically scan ros_ws/src for the declared ROS graph."
    )
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="workspace src directory (default: auto-detect a src/ with packages upward from cwd)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("graph.json"),
        help="output JSON path (default: ./graph.json)",
    )
    args = parser.parse_args(argv)

    src_root = args.src or _find_src_root(Path.cwd())
    if src_root is None or not Path(src_root).is_dir():
        print(
            "error: could not find a colcon src/ directory with packages by "
            "walking up from the current directory; pass --src /path/to/ws/src",
            file=sys.stderr,
        )
        return 2

    graph = scan_workspace(src_root)
    args.output.write_text(json.dumps(graph, indent=2) + "\n", encoding="utf-8")

    s = graph["summary"]
    print("scanned {} files under {}".format(s["files_scanned"], graph["src_root"]))
    print(
        "graph: {} nodes, {} topics, {} edges "
        "({} dynamic/unresolved topic names)".format(
            s["node_count"], s["topic_count"], s["edge_count"], s["dynamic_topic_count"]
        )
    )
    for site in s["dynamic_call_sites"]:
        print(
            "  dynamic: {}:{} ({}) topic expr: {}".format(
                site["file"], site["line"], site["kind"], site["expr"]
            )
        )
    for err in s["parse_errors"]:
        print("  parse error: {}: {}".format(err["file"], err["error"]))
    print("wrote {}".format(args.output.resolve()))
    return 0


if __name__ == "__main__":
    sys.exit(main())

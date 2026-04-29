"""
KiCad Length & Skew Constraint Monitor Plugin
Shows nets with length/skew constraints, routed lengths, and pass/fail status.
Supports sortable columns, clickable nets/classes/groups, and auto-update.
"""

import pcbnew
import wx
import wx.dataview as dv
import re
import heapq
from collections import defaultdict

# Diagnostic log for the length-calculation. Tail with:
#   tail -F /tmp/length_monitor.log
# Set LENGTH_LOG = None to disable.
LENGTH_LOG = "/tmp/length_monitor.log"

def _llog(msg):
    if not LENGTH_LOG:
        return
    try:
        with open(LENGTH_LOG, "a") as f:
            f.write(msg + "\n")
    except Exception:
        pass


# ------------------------------------------------------------------------------
#  Helpers
# ------------------------------------------------------------------------------

IU_PER_MM = pcbnew.FromMM(1)


def iu_to_mm(iu):
    return iu / IU_PER_MM


def mm_str(value_mm):
    if value_mm is None:
        return "-"
    return "{:.3f}".format(value_mm)


def _get_via_layer_range(via):
    """Return (lo, hi) copper layer IDs spanned by a PCB_VIA, trying
    multiple API method names because KiCad SWIG wrappers differ."""
    for top_name, bot_name in (("TopLayer", "BottomLayer"),
                               ("GetTopLayer", "GetBottomLayer")):
        try:
            top_fn = getattr(via, top_name, None)
            bot_fn = getattr(via, bot_name, None)
            if top_fn and bot_fn:
                top = top_fn()
                bot = bot_fn()
                if top is not None and bot is not None:
                    return (min(top, bot), max(top, bot))
        except Exception:
            continue
    # Last-ditch fallback: assume through-via F_Cu (0) → B_Cu (31).
    return (0, 31)

def _pad_copper_layers(pad):
    """Return a list of copper-only layer IDs that this pad covers.
    The pad's full layer set typically includes mask / paste / silk
    layers we don't care about for routing connectivity."""
    try:
        lset = pad.GetLayerSet()
        try:
            lset = lset & pcbnew.LSET.AllCuMask()
        except Exception:
            pass
        return list(lset.Seq())
    except Exception:
        try:
            return [pad.GetLayer()]
        except Exception:
            return []

def _bbox_contains(bbox, x, y):
    """BOX2I contains point (x, y) — handles both VECTOR2I and rect API."""
    if bbox is None:
        return False
    try:
        return bbox.Contains(pcbnew.VECTOR2I(x, y))
    except Exception:
        pass
    try:
        bx = bbox.GetX(); by = bbox.GetY()
        bw = bbox.GetWidth(); bh = bbox.GetHeight()
        return (bx <= x <= bx + bw) and (by <= y <= by + bh)
    except Exception:
        return False


def _net_name_for_code(board, net_code):
    """Look up the net name (e.g. /MCU/SDRAM/FMC_A0) for a given net code.
    Tries the direct GetNetItem(int) overload first; if that returns the
    wrong overload (str overload sometimes wins in SWIG), iterates
    NetsByName() looking for a match."""
    try:
        info = board.GetNetInfo()
    except Exception:
        return "?"
    # Direct int-overload lookup.
    try:
        ni = info.GetNetItem(net_code)
        if ni is not None:
            try:
                code = ni.GetNetCode()
            except Exception:
                code = None
            if code == net_code:
                try:
                    return str(ni.GetNetname())
                except Exception:
                    pass
    except Exception:
        pass
    # Iterate fallback.
    try:
        for name, ni in info.NetsByName().items():
            try:
                if ni.GetNetCode() == net_code:
                    return str(name)
            except Exception:
                continue
    except Exception:
        pass
    return "?"


def get_routed_length_mm(board, net_code):
    """Length of the longest pad-to-pad electrical path on the given net.
    Tries KiCad's CONNECTIVITY_DATA API first (which already knows what
    is connected to what — no geometric matching needed); if that's not
    accessible falls back to total length of all primitives."""
    if net_code <= 0:
        return 0.0

    net_name = _net_name_for_code(board, net_code)

    # First pass: collect items + total, also probe what the connectivity
    # API supports so we can pick the right code path.
    total_iu = 0
    n_tracks = n_arcs = n_vias = 0
    items_on_net = []  # all primitives on the net (tracks/arcs/vias)
    for t in board.GetTracks():
        if t.GetNetCode() != net_code:
            continue
        cls = t.GetClass()
        length = t.GetLength()
        total_iu += length
        items_on_net.append(t)
        if cls == "PCB_TRACK":
            n_tracks += 1
        elif cls == "PCB_ARC":
            n_arcs += 1
        elif cls == "PCB_VIA":
            n_vias += 1

    # Find all pads on the net.
    pads = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetCode() == net_code:
                pads.append(pad)

    if len(pads) < 2:
        # Nothing to path-find between. For unrouted internal nets KiCad
        # returns 0, so we do too — total can be 0 anyway.
        result = iu_to_mm(total_iu)
        _llog("net={:4d} {!r:<32} <2pads tr={} arc={} via={} total={:.3f} -> fallback {:.3f}"
              .format(net_code, net_name, n_tracks, n_arcs, n_vias,
                      iu_to_mm(total_iu), result))
        return result

    # Try the CONNECTIVITY_DATA-based path first.
    cd_result, n_edges = _length_via_connectivity(
        board, net_code, pads, items_on_net)
    if cd_result is not None:
        _llog("net={:4d} {!r:<32} tr={} arc={} via={} pads={} edges={} total={:.3f} -> CONN {:.3f}"
              .format(net_code, net_name, n_tracks, n_arcs, n_vias, len(pads),
                      n_edges, iu_to_mm(total_iu), cd_result))
        return cd_result

    # Last-resort fallback.
    result = iu_to_mm(total_iu)
    _llog("net={:4d} {!r:<32} tr={} arc={} via={} pads={} edges={} total={:.3f} -> CONN-FAIL fallback {:.3f}"
          .format(net_code, net_name, n_tracks, n_arcs, n_vias, len(pads),
                  n_edges, iu_to_mm(total_iu), result))
    return result


# One-shot flag so we only log the API probe once per session.
_API_PROBE_DONE = [False]


_STACKUP_PROBE_DONE = [False]


def _build_layer_depths(board):
    """Return {copper_layer_id: depth_in_iu}, where depth is measured
    from the top of the board.

    Tries (in order):
      1. BOARD_STACKUP_DESCRIPTOR with proper item enumeration — gives
         the exact dielectric thicknesses.
      2. Uniform-thickness fallback: divides total board thickness
         equally across (copper_layers − 1) dielectric gaps, mapping
         KiCad layer IDs (F.Cu=0, In1..In(N-2)=1..N-2, B.Cu=31) to
         positions 0..N-1.
    """
    if not _STACKUP_PROBE_DONE[0]:
        _STACKUP_PROBE_DONE[0] = True
        _probe_stackup_api(board)

    # ---- Attempt 1: full stackup descriptor walk --------------------
    depths = _try_stackup_walk(board)
    if depths:
        return depths

    # ---- Attempt 2: uniform-thickness fallback ----------------------
    return _uniform_stackup_depths(board)


def _try_stackup_walk(board):
    depths = {}
    try:
        stackup = board.GetDesignSettings().GetStackupDescriptor()
    except Exception:
        return depths

    # Probe a list of methods to enumerate the stackup items, since the
    # name varies between KiCad SWIG builds.
    items = None
    for fn_name in ("GetList", "GetItems", "GetStackupItems", "GetStackup"):
        fn = getattr(stackup, fn_name, None)
        if fn is None:
            continue
        try:
            r = fn()
            if r:
                items = list(r)
                break
        except Exception:
            continue
    if items is None:
        # Some builds let you index directly: stackup[i] for i in range(n).
        try:
            n = stackup.GetCount()
            items = [stackup.GetItem(i) for i in range(n)]
        except Exception:
            pass
    if not items:
        return depths

    copper_t     = getattr(pcbnew, "BS_ITEM_TYPE_COPPER",     None)
    dielectric_t = getattr(pcbnew, "BS_ITEM_TYPE_DIELECTRIC", None)

    cumulative = 0
    for it in items:
        try:
            t_type = it.GetType()
        except Exception:
            continue
        try:
            thickness = it.GetThickness() or 0
        except Exception:
            thickness = 0

        if copper_t is not None and t_type == copper_t:
            try:
                lid = it.GetBrdLayerId()
            except Exception:
                lid = None
            if lid is not None and lid >= 0:
                depths[lid] = cumulative
        elif dielectric_t is not None and t_type == dielectric_t:
            cumulative += thickness
    return depths


def _uniform_stackup_depths(board):
    """Approximation when the stackup descriptor isn't enumerable: map
    layer IDs to evenly-spaced depths across the board thickness."""
    try:
        thickness = board.GetDesignSettings().GetBoardThickness() or 0
    except Exception:
        thickness = 0
    if thickness <= 0:
        thickness = 1600000  # 1.6 mm fallback

    try:
        n_copper = board.GetCopperLayerCount() or 0
    except Exception:
        n_copper = 0
    if n_copper < 2:
        n_copper = 2

    # KiCad layer IDs: F.Cu=0, In1..In(n_copper-2)=1..(n_copper-2),
    # B.Cu=31. Map each to a position 0..n_copper-1.
    layer_ids = [0]
    for i in range(1, n_copper - 1):
        layer_ids.append(i)
    layer_ids.append(31)  # B.Cu

    per_step = thickness / (n_copper - 1)
    depths = {}
    for pos, lid in enumerate(layer_ids):
        depths[lid] = pos * per_step
    return depths


def _probe_stackup_api(board):
    """One-shot diagnostic dumping what's accessible on the stackup
    descriptor + the board's copper-layer count."""
    try:
        ds = board.GetDesignSettings()
    except Exception as ex:
        _llog("STACKUP PROBE: GetDesignSettings exc: {}".format(ex))
        return

    try:
        thickness = ds.GetBoardThickness()
    except Exception:
        thickness = "?"
    try:
        n_copper = board.GetCopperLayerCount()
    except Exception:
        n_copper = "?"
    _llog("STACKUP PROBE: BoardThickness={}  CopperLayerCount={}"
          .format(thickness, n_copper))

    try:
        stackup = ds.GetStackupDescriptor()
    except Exception as ex:
        _llog("STACKUP PROBE: GetStackupDescriptor exc: {}".format(ex))
        return

    # What methods does the stackup descriptor expose?
    try:
        attrs = sorted(m for m in dir(stackup) if not m.startswith("_"))
    except Exception:
        attrs = []
    _llog("STACKUP PROBE: dir(stackup) = " + ", ".join(attrs))

    # Try various enumeration methods
    for fn_name in ("GetList", "GetItems", "GetStackupItems", "GetStackup",
                    "GetCount"):
        fn = getattr(stackup, fn_name, None)
        if fn is None:
            continue
        try:
            r = fn()
            try:
                rlen = len(list(r)) if hasattr(r, "__iter__") else r
            except Exception:
                rlen = "?"
            _llog("STACKUP PROBE: {}() -> {}".format(fn_name, rlen))
        except Exception as ex:
            _llog("STACKUP PROBE: {}() exc {}".format(fn_name, ex))


def _probe_connectivity_api(connectivity, sample_item, type_filter):
    """Log what connectivity methods are available and which neighbour-
    enumeration call shape actually returns results. Runs once per
    plugin reload."""
    if _API_PROBE_DONE[0]:
        return
    _API_PROBE_DONE[0] = True

    methods = sorted(m for m in dir(connectivity)
                     if any(k in m.lower() for k in
                            ("connect", "net", "track", "pad", "item")))
    _llog("CONN API PROBE methods: " + ", ".join(methods))

    if sample_item is None:
        return

    sig_name = sample_item.GetClass()
    _llog("CONN API PROBE sample_item class={} netcode={}"
          .format(sig_name, sample_item.GetNetCode()))

    attempts = [
        ("GetConnectedItems(item, type_filter)",
            lambda: connectivity.GetConnectedItems(sample_item, type_filter)),
        ("GetConnectedItems(item, type_filter, False)",
            lambda: connectivity.GetConnectedItems(sample_item, type_filter, False)),
        ("GetConnectedItems(item)",
            lambda: connectivity.GetConnectedItems(sample_item)),
        ("GetConnectedTracks(item)",
            lambda: connectivity.GetConnectedTracks(sample_item)),
        ("GetConnectedPads(item)",
            lambda: connectivity.GetConnectedPads(sample_item)),
        ("GetConnectedItemsAtAnchor(item)",
            lambda: connectivity.GetConnectedItemsAtAnchor(sample_item)),
    ]
    for label, fn in attempts:
        try:
            result = fn()
            try:
                n = len(list(result))
            except Exception:
                n = "?"
            _llog("CONN API PROBE  {}  -> {} items".format(label, n))
        except Exception as ex:
            _llog("CONN API PROBE  {}  -> EXC {}".format(label, ex))


def _length_via_connectivity(board, net_code, pads, items):
    """Compute longest pad-to-pad path using BOARD.GetConnectivity().
    Returns (mm, n_edges) or (None, 0) if the API isn't usable.

    Simpler graph model than IN/OUT split:
      - Each item (track / arc / via / pad) is ONE node.
      - For each connection (u, v), edge weight = (w(u) + w(v)) / 2.
      - Path length pad_a → I1 → I2 → ... → In → pad_b sums edge weights:
            (0 + w_I1)/2 + (w_I1 + w_I2)/2 + ... + (w_In + 0)/2
          = w_I1 + w_I2 + ... + w_In  (pads add 0 to both endpoints)
    """
    try:
        connectivity = board.GetConnectivity()
    except Exception:
        return None, 0

    # KICAD_T type filter — explicit list, since [] may mean "match nothing".
    type_filter = []
    for t_name in ("PCB_TRACE_T", "PCB_VIA_T", "PCB_ARC_T", "PCB_PAD_T"):
        v = getattr(pcbnew, t_name, None)
        if v is not None:
            type_filter.append(v)

    # Probe the API once per session so the user can see exactly which
    # method shapes work in this KiCad build.
    sample = items[0] if items else (pads[0] if pads else None)
    _probe_connectivity_api(connectivity, sample, type_filter)

    # Cache board thickness + layer-depth map for via length fallback.
    # KiCad 9's SWIG build appears to return 0 from PCB_VIA.GetLength(),
    # so we synthesize the barrel length from the BOARD_STACKUP. For
    # partial-span (blind/buried) vias the depth difference between the
    # via's top and bottom copper layers is the right answer; full
    # through-vias span the entire stackup.
    _board_thickness = 0
    try:
        _board_thickness = board.GetDesignSettings().GetBoardThickness()
    except Exception:
        pass
    if not _board_thickness or _board_thickness <= 0:
        _board_thickness = 1600000  # 1.6 mm in nm (typical 4-layer)

    _layer_depths = _build_layer_depths(board)

    def _w(item):
        try:
            cls = item.GetClass()
        except Exception:
            return 0
        if cls in ("PCB_TRACK", "PCB_ARC"):
            try:
                return item.GetLength() or 0
            except Exception:
                return 0
        if cls == "PCB_VIA":
            # Try the direct API first (works on some KiCad builds).
            try:
                l = item.GetLength()
                if l and l > 0:
                    return l
            except Exception:
                pass

            # Use only the layer span ACTUALLY TRAVERSED by routed tracks
            # connected to this via. For through-vias whose stub extends
            # beyond the signal's entry/exit layers, this excludes the
            # unused stub portion. The barrel weight equals the depth
            # difference between the highest and lowest layers carrying
            # connected tracks.
            if _layer_depths:
                used_layers = set()
                try:
                    for nb in connectivity.GetConnectedTracks(item):
                        try:
                            ncls = nb.GetClass()
                        except Exception:
                            continue
                        if ncls in ("PCB_TRACK", "PCB_ARC"):
                            try:
                                used_layers.add(nb.GetLayer())
                            except Exception:
                                pass
                except Exception:
                    pass
                if len(used_layers) >= 2:
                    used_depths = [_layer_depths[L] for L in used_layers
                                   if L in _layer_depths]
                    if len(used_depths) >= 2:
                        return max(used_depths) - min(used_depths)

                # Single layer used (or no connected tracks): no traversal,
                # via contributes 0 length to this signal.
                if len(used_layers) == 1:
                    return 0

                # No track connectivity info — fall back to physical span.
                try:
                    top = item.TopLayer()
                    bot = item.BottomLayer()
                    if top in _layer_depths and bot in _layer_depths:
                        return abs(_layer_depths[bot] - _layer_depths[top])
                except Exception:
                    pass

            # Last-resort fallback: full board thickness.
            return _board_thickness
        return 0

    # Use KIID (UUID) as the stable identifier — KiCad's connectivity API
    # returns NEW Python wrapper objects every call, so id(wrapper) is not
    # stable. UUIDs are persistent across wrappers.
    def _key(item):
        for attr in ("m_Uuid",):
            u = getattr(item, attr, None)
            if u is not None:
                try:
                    return ("u", str(u.AsString()))
                except Exception:
                    try:
                        return ("u", str(u))
                    except Exception:
                        pass
        try:
            return ("u", str(item.GetUuid().AsString()))
        except Exception:
            return ("id", id(item))

    weights = {}
    item_by_id = {}
    pad_ids = set()

    def register(item):
        k = _key(item)
        if k in item_by_id:
            return k
        item_by_id[k] = item
        weights[k] = _w(item)
        return k

    for t in items:
        register(t)
    for p in pads:
        k = register(p)
        pad_ids.add(k)

    graph = defaultdict(list)
    edge_set = set()  # avoid duplicate edges

    def add_edge(u, v, w):
        if u == v:
            return
        key = (min(u, v), max(u, v))
        if key in edge_set:
            return
        edge_set.add(key)
        graph[u].append((v, w))
        graph[v].append((u, w))

    # Probe multiple call shapes — different KiCad 9 builds expose
    # different methods. Once we find a shape that returns non-empty for
    # the first item, stick with it for all subsequent items.
    def _make_query():
        method_names = (
            "GetConnectedItems",
            "GetConnectedTracks",
            "GetConnectedPads",
            "GetConnectedItemsAtAnchor",
        )
        attempts = []
        for name in method_names:
            fn = getattr(connectivity, name, None)
            if fn is None:
                continue
            if name == "GetConnectedItems":
                attempts.append((name, lambda i, _fn=fn: _fn(i, type_filter)))
                attempts.append((name + " noargs", lambda i, _fn=fn: _fn(i)))
                attempts.append((name + " 3arg",
                                 lambda i, _fn=fn: _fn(i, type_filter, False)))
            else:
                attempts.append((name, lambda i, _fn=fn: _fn(i)))

        # Combined query: returns the union of every successful call
        # shape's result. Some builds return tracks-only from
        # GetConnectedTracks and pads-only from GetConnectedPads, so we
        # need both to get a complete neighbour list.
        def query(item):
            seen_ids = set()
            results  = []
            for label, fn in attempts:
                try:
                    r = fn(item)
                except Exception:
                    continue
                if r is None:
                    continue
                try:
                    for x in r:
                        xid = id(x)
                        if xid not in seen_ids:
                            seen_ids.add(xid)
                            results.append(x)
                except Exception:
                    continue
            return results
        return query

    _query = _make_query()

    for k, item in list(item_by_id.items()):
        for nb in _query(item):
            try:
                if nb.GetNetCode() != net_code:
                    continue
            except Exception:
                pass
            nb_k = register(nb)
            edge_w = (weights[k] + weights[nb_k]) / 2.0
            add_edge(k, nb_k, edge_w)

    if len(pad_ids) < 2:
        return None, len(edge_set)

    def dijkstra(src):
        dist = {src: 0}
        heap = [(0, src)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, float('inf')):
                continue
            for v, w in graph.get(u, ()):
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    heapq.heappush(heap, (nd, v))
        return dist

    pad_id_list = list(pad_ids)
    max_iu = 0
    for i, src in enumerate(pad_id_list):
        if src not in graph:
            continue
        d = dijkstra(src)
        for tgt in pad_id_list[i+1:]:
            if tgt in d and d[tgt] > max_iu:
                max_iu = d[tgt]
    if max_iu == 0:
        return None, len(edge_set)
    return iu_to_mm(max_iu), len(edge_set)


# ------------------------------------------------------------------------------
#  DRU parser
# ------------------------------------------------------------------------------

def parse_mm_value(token):
    token = token.strip().rstrip(')')
    m = re.match(r'([\d.]+)\s*(mm|um|mil|in)?', token, re.IGNORECASE)
    if not m:
        return None
    val = float(m.group(1))
    unit = (m.group(2) or 'mm').lower()
    conversions = {'mm': 1.0, 'um': 1e-3, 'mil': 0.0254, 'in': 25.4}
    return val * conversions.get(unit, 1.0)


def parse_constraint_values(body):
    min_mm = max_mm = None
    min_m = re.search(r'\(min\s+([\d.]+\s*(?:mm|um|mil|in)?)\)', body, re.IGNORECASE)
    max_m = re.search(r'\(max\s+([\d.]+\s*(?:mm|um|mil|in)?)\)', body, re.IGNORECASE)
    opt_m = re.search(r'\(opt\s+([\d.]+\s*(?:mm|um|mil|in)?)\)', body, re.IGNORECASE)
    if min_m:
        min_mm = parse_mm_value(min_m.group(1))
    if max_m:
        max_mm = parse_mm_value(max_m.group(1))
    if opt_m and min_mm is None and max_mm is None:
        v = parse_mm_value(opt_m.group(1))
        min_mm = max_mm = v
    return min_mm, max_mm


def extract_netclasses_from_condition(condition):
    classes = set()
    for m in re.finditer(r'[AB]\.NetClass\s*==\s*[\'"]([^\'"]+)[\'"]', condition):
        classes.add(m.group(1))
    return classes


def parse_dru_rules(rules_text):
    length_rules = []
    skew_rules = []

    rule_blocks = re.findall(r'\(rule\s+"([^"]+)"(.*?)\n\)', rules_text, re.DOTALL)

    for name, body in rule_blocks:
        cond_m = re.search(r'\(condition\s+"([^"]+)"', body)
        condition = cond_m.group(1) if cond_m else ""
        classes = extract_netclasses_from_condition(condition)

        # length constraint
        len_m = re.search(r'\(constraint\s+length([^)]*\([^)]*\))', body, re.DOTALL)
        if len_m:
            min_mm, max_mm = parse_constraint_values(len_m.group(1))
            if min_mm is not None or max_mm is not None:
                length_rules.append({
                    'rule_name': name,
                    'condition': condition,
                    'classes':   classes,
                    'min_mm':    min_mm,
                    'max_mm':    max_mm,
                })

        # skew constraint
        skew_m = re.search(r'\(constraint\s+skew([^)]*\([^)]*\))', body, re.DOTALL)
        if skew_m:
            _, max_skew = parse_constraint_values(skew_m.group(1))
            if max_skew is not None:
                skew_rules.append({
                    'rule_name':   name,
                    'condition':   condition,
                    'classes':     classes,
                    'max_skew_mm': max_skew,
                })

    return length_rules, skew_rules


def read_dru_text(board):
    board_path = board.GetFileName()
    if not board_path:
        return ""
    dru_path = board_path.replace('.kicad_pcb', '.kicad_dru')
    try:
        with open(dru_path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception:
        return ""


# ------------------------------------------------------------------------------
#  Net -> class mapping  (KiCad 9 compatible)
# ------------------------------------------------------------------------------

def build_net_to_class(board):
    """
    Return dict net_name -> primary_class_name for all nets.
    KiCad 9 returns composite names like 'ADDR,Default' from
    GetEffectiveNetClass().GetName() -- we take the first token.
    Falls back to old GetNetClasses() API for KiCad 7/8.
    """
    net_to_class = {}
    net_info = board.GetNetInfo()

    # KiCad 9 path
    try:
        ns = board.GetDesignSettings().m_NetSettings
        for net_name in net_info.NetsByName():
            if not net_name:
                continue
            try:
                composite = str(ns.GetEffectiveNetClass(net_name).GetName())
                primary = composite.split(',')[0].strip()
                net_to_class[str(net_name)] = primary
            except Exception:
                net_to_class[str(net_name)] = "Default"
        return net_to_class
    except Exception:
        pass

    # KiCad 7/8 fallback
    for net_name in net_info.NetsByName():
        net_to_class[net_name] = "Default"

    nc_obj = board.GetNetClasses()
    try:
        items = list(nc_obj.items())
    except AttributeError:
        try:
            items = list(nc_obj.NetClasses().items())
        except Exception:
            items = []

    for class_name, netclass in items:
        try:
            for i in range(netclass.GetCount()):
                net_to_class[netclass.GetMember(i)] = class_name
        except Exception:
            pass

    return net_to_class


# ------------------------------------------------------------------------------
#  Table columns
# ------------------------------------------------------------------------------

COL_NET      = 0
COL_CLASS    = 1
COL_LEN      = 2
COL_MIN      = 3
COL_MAX      = 4
COL_SKEW_GRP = 5
COL_MAX_SKEW = 6
COL_ACT_SKEW = 7
COL_STATUS   = 8

COLUMNS    = ["Net Name",  "Net Class", "Routed (mm)", "Min (mm)", "Max (mm)",
              "Skew Group", "Max Skew",  "Act. Skew",   "Status"]
COL_WIDTHS = [200, 110, 100, 85, 85, 160, 80, 80, 80]

NUMERIC_COLS = {COL_LEN, COL_MIN, COL_MAX, COL_MAX_SKEW, COL_ACT_SKEW}


# ------------------------------------------------------------------------------
#  Data row
# ------------------------------------------------------------------------------

class NetRow(object):
    def __init__(self, net_name, class_name, length_mm,
                 min_mm, max_mm, skew_group, max_skew_mm,
                 group_min_mm, group_max_mm):
        self.net_name      = net_name
        self.class_name    = class_name
        self.length_mm     = length_mm
        self.min_mm        = min_mm
        self.max_mm        = max_mm
        self.skew_group    = skew_group
        self.max_skew_mm   = max_skew_mm
        # group_min_mm / group_max_mm = shortest / longest length across all
        # nets in the same skew group. Stored on each row so per-row sort
        # and per-row skew display stay row-local. Pass/fail still uses the
        # group's spread (max - min) — that's a single value across the
        # group, not per net.
        self.group_min_mm  = group_min_mm
        self.group_max_mm  = group_max_mm

    @property
    def actual_skew_mm(self):
        """This net's distance from the farther of the two group extremes:
            actual_skew = max(|length - group_max|, |length - group_min|)
        - Shortest and longest nets both report the full group spread
          (they each sit at one extreme).
        - Mid-group nets report the distance to whichever extreme is
          farther — a "worst-case mismatch with the rest of the group"
          metric. Useful for ranking outliers.
        """
        if self.group_min_mm is None or self.group_max_mm is None:
            return None
        return max(abs(self.length_mm - self.group_max_mm),
                   abs(self.length_mm - self.group_min_mm))

    @property
    def group_skew_mm(self):
        """Group spread = max - min across all nets in the skew group.
        Same value for every net in the group; used for pass/fail check."""
        if self.group_min_mm is None or self.group_max_mm is None:
            return None
        return self.group_max_mm - self.group_min_mm

    @property
    def length_ok(self):
        if self.min_mm is not None and self.length_mm < self.min_mm:
            return False
        if self.max_mm is not None and self.length_mm > self.max_mm:
            return False
        return True

    @property
    def skew_ok(self):
        if self.max_skew_mm is None or self.group_skew_mm is None:
            return True
        # Group spread (max - min) must satisfy the constraint. If the
        # group fails, every net in the group is flagged FAIL.
        return self.group_skew_mm <= self.max_skew_mm

    @property
    def is_ok(self):
        return self.length_ok and self.skew_ok

    @property
    def status(self):
        if self.is_ok:
            return "OK"
        parts = []
        if not self.length_ok:
            parts.append("LEN")
        if not self.skew_ok:
            parts.append("SKEW")
        return "FAIL: " + "+".join(parts)

    def get_col(self, col):
        if col == COL_NET:      return self.net_name
        if col == COL_CLASS:    return self.class_name
        if col == COL_LEN:      return "{:.3f}".format(self.length_mm)
        if col == COL_MIN:      return mm_str(self.min_mm)
        if col == COL_MAX:      return mm_str(self.max_mm)
        if col == COL_SKEW_GRP: return self.skew_group
        if col == COL_MAX_SKEW: return mm_str(self.max_skew_mm)
        if col == COL_ACT_SKEW: return mm_str(self.actual_skew_mm)
        if col == COL_STATUS:   return self.status
        return ""


# ------------------------------------------------------------------------------
#  Main data builder
# ------------------------------------------------------------------------------

def build_rows(board):
    rules_text = read_dru_text(board)
    if not rules_text:
        return []

    length_rules, skew_rules = parse_dru_rules(rules_text)
    net_to_class = build_net_to_class(board)
    net_info = board.GetNetInfo()

    # Pre-compute routed lengths
    net_lengths = {}
    for net_name, ni in net_info.NetsByName().items():
        net_name = str(net_name)
        if net_name:
            net_lengths[net_name] = get_routed_length_mm(board, ni.GetNetCode())

    # Accumulate constraints per net
    net_data = {}

    def ensure(net_name):
        net_name = str(net_name)  # convert wxString to Python str
        if net_name not in net_data:
            net_data[net_name] = {
                'min_mm':      None,
                'max_mm':      None,
                'skew_group':  "",
                'max_skew_mm': None,
            }

    for rule in length_rules:
        for cls in rule['classes']:
            for net_name, nc in net_to_class.items():
                if nc == cls:
                    ensure(str(net_name))
                    d = net_data[net_name]
                    if rule['min_mm'] is not None:
                        d['min_mm'] = max(d['min_mm'], rule['min_mm']) \
                            if d['min_mm'] is not None else rule['min_mm']
                    if rule['max_mm'] is not None:
                        d['max_mm'] = min(d['max_mm'], rule['max_mm']) \
                            if d['max_mm'] is not None else rule['max_mm']

    for rule in skew_rules:
        for cls in rule['classes']:
            for net_name, nc in net_to_class.items():
                if nc == cls:
                    ensure(str(net_name))
                    d = net_data[net_name]
                    if d['max_skew_mm'] is None or \
                       rule['max_skew_mm'] < d['max_skew_mm']:
                        d['max_skew_mm'] = rule['max_skew_mm']
                        d['skew_group']  = rule['rule_name']

    if not net_data:
        return []

    # Compute group min and max so each NetRow can carry a per-net actual
    # skew (length - group_min) AND we can still compute group spread
    # (group_max - group_min) for pass/fail.
    group_lengths = defaultdict(list)
    for net_name, d in net_data.items():
        if d['skew_group']:
            group_lengths[d['skew_group']].append(net_lengths.get(net_name, 0.0))

    group_min = {}
    group_max = {}
    for g, lens in group_lengths.items():
        if lens:
            group_min[g] = min(lens)
            group_max[g] = max(lens)

    # Build rows
    rows = []
    for net_name, d in sorted(net_data.items()):
        grp = d['skew_group']
        gmin = group_min.get(grp) if grp else None
        gmax = group_max.get(grp) if grp else None
        rows.append(NetRow(
            net_name     = net_name,
            class_name   = net_to_class.get(net_name, "Default"),
            length_mm    = net_lengths.get(net_name, 0.0),
            min_mm       = d['min_mm'],
            max_mm       = d['max_mm'],
            skew_group   = grp,
            max_skew_mm  = d['max_skew_mm'],
            group_min_mm = gmin,
            group_max_mm = gmax,
        ))

    return rows


# ------------------------------------------------------------------------------
#  DataViewModel
# ------------------------------------------------------------------------------

class NetTableModel(dv.DataViewIndexListModel):
    def __init__(self):
        super(NetTableModel, self).__init__(0)
        self.rows = []

    def GetColumnCount(self):
        return len(COLUMNS)

    def GetColumnType(self, col):
        return "string"

    def GetValueByRow(self, row, col):
        if row >= len(self.rows):
            return ""
        return self.rows[row].get_col(col)

    def GetAttrByRow(self, row, col, attr):
        if row >= len(self.rows):
            return False
        if not self.rows[row].is_ok:
            attr.SetColour(wx.Colour(200, 50, 50))
            attr.SetBold(True)
            return True
        return False

    def SetValueByRow(self, value, row, col):
        return False

    def GetCount(self):
        return len(self.rows)

    def Compare(self, item1, item2, col, ascending):
        r1 = self.GetRow(item1)
        r2 = self.GetRow(item2)
        if r1 >= len(self.rows) or r2 >= len(self.rows):
            return 0
        v1 = self.rows[r1].get_col(col)
        v2 = self.rows[r2].get_col(col)
        if col in NUMERIC_COLS:
            try:
                v1 = float(v1.replace('-', '-1'))
                v2 = float(v2.replace('-', '-1'))
            except ValueError:
                pass
        result = (v1 > v2) - (v1 < v2)
        return result if ascending else -result

    def refresh(self, new_rows):
        # If the row COUNT is unchanged, notify per-row updates instead of
        # calling Reset(). Reset() invalidates the dvc's selection and scroll
        # position, which makes auto-refresh disruptive while the user is
        # interacting. RowChanged() keeps both intact and only redraws the
        # affected cells.
        old_rows = self.rows
        old_count = len(old_rows)
        new_count = len(new_rows)
        self.rows = new_rows
        if old_count != new_count:
            self.Reset(new_count)
            return
        # Per-row diff: only notify on rows whose visible data changed.
        for i in range(new_count):
            if self._row_equal(old_rows[i], new_rows[i]):
                continue
            try:
                self.RowChanged(i)
            except Exception:
                # If RowChanged isn't available on this wx build, fall back
                # to a single Reset (still better than the loop crashing).
                self.Reset(new_count)
                return

    @staticmethod
    def _row_equal(a, b):
        """True if two NetRow objects look identical to the user (same
        values in every visible column). Used to skip unnecessary
        RowChanged notifications."""
        return (a.net_name     == b.net_name
            and a.class_name    == b.class_name
            and a.length_mm     == b.length_mm
            and a.min_mm        == b.min_mm
            and a.max_mm        == b.max_mm
            and a.skew_group    == b.skew_group
            and a.max_skew_mm   == b.max_skew_mm
            and a.group_min_mm  == b.group_min_mm
            and a.group_max_mm  == b.group_max_mm)


# ------------------------------------------------------------------------------
#  Dialog
# ------------------------------------------------------------------------------

class LengthMonitorDialog(wx.Frame):
    POLL_MS     = 2000   # length-table refresh
    SEL_POLL_MS = 250    # reverse-sync: pick up PCB selection changes

    def __init__(self, board):
        super(LengthMonitorDialog, self).__init__(
            None,
            title="Length & Skew Constraint Monitor (v3.3 - stackup uniform)",
            size=(980, 520),
            style=wx.DEFAULT_FRAME_STYLE | wx.STAY_ON_TOP
        )
        self.board                = board
        self._last_hash           = None
        self._timer               = None
        self._sel_timer           = None
        self._all_rows            = []
        self._last_click_pos      = None  # cached for column lookup on selection
        self._last_pcb_netcode    = None  # last single-net code seen on PCB
        self._last_applied_codes  = None  # cached arg from _apply_selection
        self._suppress_sel_change = False # block our own dvc.Select feedback

        self._build_ui()
        self._refresh()
        self._start_timer()
        self.Show()

    def _build_ui(self):
        panel = wx.Panel(self)
        sizer = wx.BoxSizer(wx.VERTICAL)

        tb = wx.BoxSizer(wx.HORIZONTAL)
        self.lbl_status = wx.StaticText(panel, label="")
        btn_refresh = wx.Button(panel, label="Refresh", size=(80, -1))
        btn_refresh.Bind(wx.EVT_BUTTON, lambda e: self._refresh())
        self.chk_auto = wx.CheckBox(panel, label="Auto-update")
        self.chk_auto.SetValue(True)
        self.chk_auto.Bind(wx.EVT_CHECKBOX, self._on_auto_toggle)
        tb.Add(self.lbl_status, 1, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 6)
        tb.Add(self.chk_auto,   0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        tb.Add(btn_refresh,     0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
        sizer.Add(tb, 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 4)

        self.model = NetTableModel()
        self.dvc = dv.DataViewCtrl(
            panel,
            style=dv.DV_ROW_LINES | dv.DV_VERT_RULES | dv.DV_SINGLE
        )
        self.dvc.AssociateModel(self.model)
        # Do NOT call DecRef() -- causes C++ object deleted errors in KiCad 9

        for i, (col_name, width) in enumerate(zip(COLUMNS, COL_WIDTHS)):
            self.dvc.AppendTextColumn(
                col_name, i,
                width=width,
                mode=dv.DATAVIEW_CELL_ACTIVATABLE,
                flags=dv.DATAVIEW_COL_SORTABLE | dv.DATAVIEW_COL_RESIZABLE
            )

        # Single-click path: EVT_LEFT_DOWN captures the click position (so
        # we know which column was clicked); EVT_DATAVIEW_SELECTION_CHANGED
        # fires after the widget updates its selection, so we read the
        # correct row from GetSelection(). Keyboard up/down arrows also
        # trigger SELECTION_CHANGED, so they "just work" without extra code.
        self.dvc.Bind(wx.EVT_LEFT_DOWN,                self._on_left_down)
        self.dvc.Bind(dv.EVT_DATAVIEW_SELECTION_CHANGED, self._on_selection_changed)
        sizer.Add(self.dvc, 1, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)

        fb = wx.BoxSizer(wx.HORIZONTAL)
        fb.Add(wx.StaticText(panel, label="Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 6)
        self.txt_filter = wx.TextCtrl(panel, size=(180, -1))
        self.txt_filter.Bind(wx.EVT_TEXT, self._on_filter)
        self.chk_fail = wx.CheckBox(panel, label="Failures only")
        self.chk_fail.Bind(wx.EVT_CHECKBOX, self._on_filter)
        fb.Add(self.txt_filter, 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT | wx.RIGHT, 6)
        fb.Add(self.chk_fail,   0, wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(fb, 0, wx.EXPAND | wx.BOTTOM, 6)

        panel.SetSizer(sizer)
        self.Bind(wx.EVT_CLOSE, self._on_close)

    def _apply_filter(self, rows):
        txt       = self.txt_filter.GetValue().lower()
        fail_only = self.chk_fail.GetValue()
        result = []
        for r in rows:
            if txt and txt not in r.net_name.lower() \
                   and txt not in r.class_name.lower() \
                   and txt not in r.skew_group.lower():
                continue
            if fail_only and r.is_ok:
                continue
            result.append(r)
        return result

    def _board_hash(self):
        try:
            # Include arcs and vias too, not just straight tracks — they all
            # contribute to length and any change should trigger a refresh.
            return hash(tuple(
                (t.GetNetCode(), t.GetLength())
                for t in self.board.GetTracks()
            ))
        except Exception:
            return None

    def _refresh(self):
        try:
            # Capture current dvc selection by NET NAME so it can be
            # restored after refresh even if the row count or order changes.
            saved_net = None
            try:
                sel_item = self.dvc.GetSelection()
                if sel_item is not None and sel_item.IsOk():
                    sel_idx = self.model.GetRow(sel_item)
                    if 0 <= sel_idx < len(self.model.rows):
                        saved_net = self.model.rows[sel_idx].net_name
            except Exception:
                pass

            self._all_rows = build_rows(self.board)
            filtered = self._apply_filter(self._all_rows)
            self.model.refresh(filtered)
            fails = sum(1 for r in filtered if not r.is_ok)
            self.lbl_status.SetLabel(
                "{} nets  |  {} failing{}".format(
                    len(filtered), fails, "  !" if fails else "")
            )
            self._last_hash = self._board_hash()

            # Restore selection by net name if we had one.
            if saved_net is not None:
                for idx, row in enumerate(self.model.rows):
                    if row.net_name == saved_net:
                        try:
                            item = self.model.GetItem(idx)
                            self._suppress_sel_change = True
                            self.dvc.Select(item)
                            self.dvc.EnsureVisible(item)
                            wx.CallAfter(self._reset_suppress)
                        except Exception:
                            pass
                        break
        except Exception as ex:
            import traceback
            self.lbl_status.SetLabel("Error: {}".format(ex))
            print(traceback.format_exc())

    def _dispatch_activation(self, row, col_idx):
        """Run the right selection action based on which column was hit."""
        if col_idx == COL_CLASS and row.class_name:
            self._select_netclass(row.class_name)
        elif col_idx == COL_SKEW_GRP and row.skew_group:
            self._select_skew_group(row.skew_group)
        else:
            self._select_net(row.net_name)

    def _on_left_down(self, event):
        # Cache the click position so _on_selection_changed can determine
        # which column was clicked via HitTest. Skip() to let the widget
        # process the click (update its selection, etc.).
        self._last_click_pos = event.GetPosition()
        event.Skip()

    def _on_selection_changed(self, event):
        # Fired after the dvc updates its row selection (mouse single-click
        # OR keyboard up/down). GetSelection() returns the new row; the
        # column comes from HitTest of the cached click position (or -1
        # for keyboard navigation, which falls through to single-net
        # selection — the right default for keyboard users).
        if self._suppress_sel_change:
            # Programmatic update from _sync_table_to_net — skip handler.
            return
        sel_item = self.dvc.GetSelection()
        if sel_item is None or not sel_item.IsOk():
            return
        row_idx = self.model.GetRow(sel_item)
        if row_idx < 0 or row_idx >= len(self.model.rows):
            return
        row = self.model.rows[row_idx]

        col_idx = -1
        if self._last_click_pos is not None:
            try:
                _, col = self.dvc.HitTest(self._last_click_pos)
                if col is not None:
                    try:
                        col_idx = col.GetModelColumn()
                    except Exception:
                        try:
                            col_idx = self.dvc.GetColumnPosition(col)
                        except Exception:
                            col_idx = -1
            except Exception:
                pass
            # Consume the cached position so a subsequent keyboard-driven
            # selection change doesn't reuse a stale mouse column.
            self._last_click_pos = None

        self._dispatch_activation(row, col_idx)

    def _get_pcb_frame(self):
        """Find the PCB Editor wx top-level window. KiCad 9 dropped
        pcbnew.GetMainFrame(); we hunt through wx's top-level windows for
        a frame that exposes GetToolManager()."""
        # Try the legacy API first in case it's present on older builds.
        try:
            fn = getattr(pcbnew, "GetMainFrame", None)
            if fn:
                f = fn()
                if f is not None:
                    return f
        except Exception:
            pass
        # wx fallback: look for any top-level window with a ToolManager.
        try:
            for win in wx.GetTopLevelWindows():
                if hasattr(win, "GetToolManager"):
                    return win
        except Exception:
            pass
        return None

    def _find_selection_tool(self):
        """Locate KiCad's PCB_SELECTION_TOOL (name varies between versions)."""
        frame = self._get_pcb_frame()
        if frame is None:
            return None, None
        try:
            tm = frame.GetToolManager()
        except Exception:
            return None, None
        for tool_id in ("pcbnew.InteractiveSelection",
                        "common.InteractiveSelection",
                        "PCB_SELECTION_TOOL"):
            try:
                tool = tm.FindTool(tool_id)
                if tool is not None:
                    return tm, tool
            except Exception:
                continue
        return tm, None

    def _apply_selection(self, net_codes):
        """Drive the SELECTION_TOOL (not the items directly) so KiCad doesn't
        revert our changes on the next event loop. Falls back to direct
        SetSelected/ClearSelected if the tool isn't reachable."""
        # Skip if we've just applied this exact set — avoids loops with the
        # reverse-sync path (PCB selection -> dvc.Select -> SELECTION_CHANGED
        # -> _apply_selection -> ... back to PCB selection same as before).
        codes_set = frozenset(net_codes)
        if codes_set == self._last_applied_codes:
            return
        self._last_applied_codes = codes_set
        # Also update the reverse-sync's last-known PCB code so we don't
        # immediately re-fire from our own update.
        self._last_pcb_netcode = (next(iter(codes_set))
                                  if len(codes_set) == 1 else None)

        tracks_to_select = [t for t in self.board.GetTracks()
                            if t.GetNetCode() in net_codes]

        tm, sel_tool = self._find_selection_tool()
        ok = False
        if tm is not None:
            # Step 1: clear via the tool action (updates its internal list).
            # KiCad action names are case-sensitive; try both casings.
            for clear_action in ("common.InteractiveSelection.ClearSelection",
                                 "common.InteractiveSelection.clearSelection",
                                 "pcbnew.InteractiveSelection.ClearSelection",
                                 "pcbnew.InteractiveSelection.clearSelection"):
                try:
                    tm.RunAction(clear_action, True)
                    ok = True
                    break
                except Exception:
                    continue

        # Step 2: add tracks to the tool's selection.
        if ok and sel_tool is not None:
            added = False
            for track in tracks_to_select:
                # Try the API methods exposed by KiCad 9 SELECTION_TOOL.
                for method_name in ("AddItemToSel", "select"):
                    try:
                        getattr(sel_tool, method_name)(track)
                        added = True
                        break
                    except Exception:
                        continue
                if not added:
                    # Fall back to direct flag for this track.
                    track.SetSelected()
        else:
            # No tool route worked — direct flag manipulation as last resort.
            for track in self.board.GetTracks():
                if track.GetNetCode() in net_codes:
                    track.SetSelected()
                else:
                    track.ClearSelected()

        self._apply_highlight(net_codes)
        pcbnew.Refresh()

    def _apply_highlight(self, net_codes):
        """Highlight the given net codes on the BOARD (dims all other nets).
        BOARD-level API works even without frame access. Multi-net highlight
        uses SetHighLightNet(code, True); falls back to single-net if that
        signature isn't available in this KiCad build."""
        try:
            # Clear current highlight set
            if hasattr(self.board, "ResetNetHighLight"):
                self.board.ResetNetHighLight()
            elif hasattr(self.board, "HighLightOFF"):
                self.board.HighLightOFF()

            if not net_codes:
                return

            codes = list(net_codes)
            try:
                # Try the multi-arg overload first; if it works, use it for all.
                self.board.SetHighLightNet(codes[0], True)
                for code in codes[1:]:
                    self.board.SetHighLightNet(code, True)
            except TypeError:
                # Single-arg only — can highlight just one net.
                self.board.SetHighLightNet(codes[0])

            if hasattr(self.board, "HighLightON"):
                self.board.HighLightON()
        except Exception:
            # Highlight is best-effort; never break selection on failure.
            pass

    def _clear_highlight(self):
        """Remove any active highlight (called on dialog close)."""
        try:
            if hasattr(self.board, "ResetNetHighLight"):
                self.board.ResetNetHighLight()
            if hasattr(self.board, "HighLightOFF"):
                self.board.HighLightOFF()
            pcbnew.Refresh()
        except Exception:
            pass

    # NOTE: a "zoom-to-selection" path was tried and removed. KiCad 9 dropped
    # the Python bindings that exposed PCB_EDIT_FRAME's ToolManager / Canvas /
    # View, so a plugin can't drive the viewport. The wx top-level window for
    # the PCB Editor is now a plain wxFrame with no KiCad-specific methods.
    # Workaround for the user: bind a hotkey to "Zoom to Objects" or "Fit
    # Selection" in KiCad → Preferences → Hotkeys, then press it after the
    # plugin has selected the nets (selection works correctly via the
    # SELECTION_TOOL).

    def _select_net(self, net_name):
        try:
            ni = self.board.GetNetInfo().GetNetItem(net_name)
            if ni is None:
                return
            self._apply_selection({ni.GetNetCode()})
        except Exception:
            import traceback
            wx.MessageBox(traceback.format_exc(), "Selection Error")

    def _select_netclass(self, class_name):
        try:
            net_to_class = build_net_to_class(self.board)
            net_info = self.board.GetNetInfo()
            net_codes = set()
            for name, ni in net_info.NetsByName().items():
                if net_to_class.get(name) == class_name:
                    net_codes.add(ni.GetNetCode())
            self._apply_selection(net_codes)
        except Exception:
            import traceback
            wx.MessageBox(traceback.format_exc(), "Selection Error")

    def _select_skew_group(self, skew_group):
        try:
            net_info = self.board.GetNetInfo()
            net_codes = set()
            for r in self._all_rows:
                if r.skew_group == skew_group:
                    ni = net_info.GetNetItem(r.net_name)
                    if ni:
                        net_codes.add(ni.GetNetCode())
            self._apply_selection(net_codes)
        except Exception:
            import traceback
            wx.MessageBox(traceback.format_exc(), "Selection Error")

    def _on_filter(self, event):
        filtered = self._apply_filter(self._all_rows)
        self.model.refresh(filtered)

    def _on_auto_toggle(self, event):
        if self.chk_auto.GetValue():
            self._start_timer()
        else:
            self._stop_timer()

    def _on_close(self, event):
        self._stop_timer()
        self._stop_sel_timer()
        self._clear_highlight()
        self.Destroy()

    def _start_timer(self):
        if self._timer is None:
            self._timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self._on_timer, self._timer)
        self._timer.Start(self.POLL_MS)
        # Use a separate, faster timer for reverse-sync: pick up changes in
        # the PCB editor's selection so clicking a track on the board scrolls
        # / highlights its row in our table.
        if self._sel_timer is None:
            self._sel_timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self._on_sel_timer, self._sel_timer)
        self._sel_timer.Start(self.SEL_POLL_MS)

    def _stop_timer(self):
        if self._timer:
            self._timer.Stop()

    def _stop_sel_timer(self):
        if self._sel_timer:
            self._sel_timer.Stop()

    def _on_timer(self, event):
        if self._board_hash() != self._last_hash:
            self._refresh()

    def _on_sel_timer(self, event):
        self._check_pcb_selection()

    def _check_pcb_selection(self):
        """Look at what's currently selected on the PCB and, if it boils
        down to a single net, sync our table to that row."""
        codes = set()
        for track in self.board.GetTracks():
            if track.IsSelected():
                codes.add(track.GetNetCode())

        # Reverse sync only when the PCB selection collapses to one net.
        # Multi-net PCB selections (e.g. user drag-selected a region) are
        # ambiguous — we don't try to pick a row.
        new_code = next(iter(codes)) if len(codes) == 1 else None

        if new_code == self._last_pcb_netcode:
            return  # no change since last poll
        self._last_pcb_netcode = new_code
        if new_code is None:
            return
        self._sync_table_to_net(new_code)

    def _sync_table_to_net(self, net_code):
        """Find the dvc row for the given net code and scroll/select it.
        Wraps the dvc.Select call with a suppress flag so the resulting
        SELECTION_CHANGED event doesn't bounce back as a redundant
        _apply_selection call."""
        for idx, row in enumerate(self.model.rows):
            try:
                ni = self.board.GetNetInfo().GetNetItem(row.net_name)
            except Exception:
                continue
            if ni is None or ni.GetNetCode() != net_code:
                continue

            try:
                item = self.model.GetItem(idx)
            except Exception:
                continue

            self._suppress_sel_change = True
            try:
                self.dvc.UnselectAll()
                self.dvc.Select(item)
                self.dvc.EnsureVisible(item)
            except Exception:
                pass
            # Clear the suppress flag after the current event chain so the
            # SELECTION_CHANGED triggered by Select() above is ignored.
            wx.CallAfter(self._reset_suppress)
            return

    def _reset_suppress(self):
        self._suppress_sel_change = False


# ------------------------------------------------------------------------------
#  Plugin entry point
# ------------------------------------------------------------------------------

class LengthMonitorPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name             = "Length Constraint Monitor"
        self.category         = "PCB Analysis"
        self.description      = (
            "Shows nets with length/skew constraints, routed lengths, "
            "and pass/fail status. Click nets/classes/groups to select them."
        )
        self.show_toolbar_button = True
        self.icon_file_name      = ""

    def Run(self):
        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("No board loaded.", "Length Monitor")
            return
        LengthMonitorDialog(board)


def register():
    LengthMonitorPlugin().register()

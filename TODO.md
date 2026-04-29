# Length Monitor — TODO / Known Issues

Tracked from bench-testing in KiCad 9. Each entry has the symptom and what
we already know about the cause.

---

## 1. Auto-update makes the main UI unresponsive

**Symptom**: With "Auto-update" checked, the PCB Editor's mouse / keyboard
become laggy. Manual refresh works fine.

**What's known**:
- v1.5 fixed the obvious issue (the auto-refresh was calling `Reset()` on
  the model, which blew away the dvc's selection and scroll position).
  Now we use per-row `RowChanged()` notifications and only on rows whose
  data actually differs.
- The PCB-side selection poll runs every 250 ms and walks every track via
  `track.IsSelected()`. On a board with thousands of tracks this is
  cheap but not free.
- The 2 s table-refresh poll calls `build_rows(board)` which:
  - parses `.kicad_dru` (cheap once, but happens every poll)
  - walks every track to compute per-net length (`get_routed_length_mm`)
  - rebuilds the netclass mapping
- Even when nothing changed, those allocations still run every 2 s.

**Likely fixes to try**:
- Cache the parsed DRU rules across polls; only re-parse when the file
  mtime changes.
- Cache the netclass mapping; rebuild only when the BOARD's net-settings
  hash changes.
- Move the length computation into a worker thread (`threading.Thread`)
  and post results back via `wx.CallAfter` so the GUI thread isn't
  blocked during the crunch.
- Or: use a coarser hash check at the top of `_refresh()` so we early-out
  before doing any allocation when truly nothing changed (current
  `_board_hash()` runs but might be slower than the work it saves).
- Worst case: drop the auto-update poll entirely; rely on a manual
  Refresh button + a faster on-demand recompute path.

---

## 2. Highlight net does not work

**Symptom**: After clicking a row, tracks select but the BOARD-level
"highlight net" effect (dim everything else, brighten the selected nets)
doesn't render.

**What's known**:
- v1.3 added a call chain:
  `board.ResetNetHighLight()` →
  `board.SetHighLightNet(code, multi=True)` per net →
  `board.HighLightON()` → `pcbnew.Refresh()`
- KiCad's highlight is a render-settings effect that lives on the
  `KIGFX::RENDER_SETTINGS` attached to the canvas's view, not directly
  on the BOARD. Setting flags on `BOARD` may update the data model but
  if the canvas doesn't re-read those flags the visual effect won't
  show.
- We can't reach the canvas/view from Python (KiCad 9 dropped those
  bindings — same root cause as the zoom problem).

**Likely fixes to try**:
- Probe whether `BOARD.SetHighLightNet` takes a second arg in this
  build (multi-net mode). If `TypeError`, fall back to single-net.
- Inspect what attrs `BOARD` exposes — is there a `SetHighLightNetCodes`
  list-form variant, or a `HighLightNet(code)` (no underscore) that
  also pushes to render settings?
- Try `pcbnew.Refresh()` *after* `HighLightON()` in case the order
  matters. Also try a ProcessEvent / call to invalidate the canvas
  through any wx route we can find.
- If render-settings really is the only path: may not be fixable from
  Python in KiCad 9. Document the limitation.

---

## 3. Selection of the entire class does not work

**Symptom**: Clicking a "Net Class" cell selects only the single row's
net rather than every net in that class.

**What's known**:
- The dispatch logic (`_dispatch_activation`) checks
  `if col_idx == COL_CLASS and row.class_name: _select_netclass(...)`.
- `col_idx` comes from `HitTest(self._last_click_pos)` which is captured
  in `_on_left_down`. The wxDataViewCtrl HitTest off-by-one we hit
  earlier was for the ROW; the COLUMN field was supposedly correct.
  But this needs re-verification — maybe the column also drifts on
  GTK and we're getting `col_idx == 0` or `-1` even when the user
  clicked the Class cell.
- Could also be that `_select_netclass` itself runs but the netclass
  matching loop doesn't find any nets (e.g. the comparison
  `net_to_class.get(name) == class_name` fails because of the
  composite "CLASS,Default" string handling).

**Likely fixes to try**:
- Add a brief diagnostic popup that shows `col_idx`, `row.class_name`,
  and the count of nets matching the class — this will pin down which
  half is broken in <1 click.
- Re-test HitTest's column reliability by clicking deliberately in
  each column with the diagnostic enabled.
- Confirm `build_net_to_class()` returns clean class names (no
  composite strings) so the equality test in `_select_netclass`
  matches what's stored on each row.

---

## 4. T-junction length: report longest pad-to-pad path, not the sum

**Symptom**: For a net routed with a T-junction (one source → two or more
loads via a branch), the plugin reports the SUM of all track lengths, but
KiCad's net inspector reports the longest pad-to-pad path. Concrete case:
ADDR signal, plugin = 94.5 mm, KiCad = 82.5 mm — the 12 mm difference is
the shorter stub arm.

**What we tried (v1.9, reverted in v2.0)**: A Dijkstra over a graph with
nodes = `(x, y, layer)` and edges = tracks/arcs/vias. Result: every net
in the ADDR group reported ~19.7 mm, far short of the actual route.
Likely root cause: track endpoints don't land exactly on pad centers (KiCad
allows a track to terminate on the pad's edge or via center, not its
geometric center), so the graph has disconnected components and Dijkstra
finds only a small reachable sub-graph from each pad. The 0.05 mm snap
tolerance was too tight; bumping it would cause false connections between
distinct nets / layers.

**What KiCad does internally**: full geometric overlap testing in C++
using `SHAPE` objects. Not reachable from Python in KiCad 9.

**Likely fixes to try**:
- Use `BOARD.GetConnectivity()` (CONNECTIVITY_DATA) — it might expose a
  `GetNetItems(netcode)` that yields a clean per-net list including the
  topology graph KiCad already computed. If yes, traversing that
  structure avoids re-doing geometric tests in Python.
- Build the graph from the connectivity object instead of from raw
  tracks, so endpoint↔pad linkage is already correct.
- If CONNECTIVITY_DATA isn't usable: implement a BBox-based pad↔track
  link (each pad has a bbox; any track endpoint inside it gets a 0-cost
  edge to the pad node). More expensive but topologically correct.

**Current state (v2.0)**: reverted to total-length sum. Matches KiCad
exactly for point-to-point nets. Overstates for branched nets — the user
will see this as "plugin > KiCad" by exactly the stub length(s).

---

## Other ideas (lower priority)

- Group Mean column alongside Group Skew (max-min already there).
- "Failures only" filter checkbox — already in the UI; verify it works
  with the new per-row diff refresh path.
- Right-click context menu on a row (Copy net name, Open net in
  Find dialog, etc.).
- Persist column widths / sort order in `~/.config/kicad/length_monitor.json`.
- Bind a configurable hotkey (in the dialog) so the user can click a row
  and then press a key to manually trigger their own KiCad zoom-fit-
  selection without leaving the dialog.

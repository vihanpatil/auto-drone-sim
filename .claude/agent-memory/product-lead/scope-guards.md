---
name: scope-guards
description: Standing product-lead scope guard for rest of project + the two Weeks 5-6 cuts (no YOLO, no startup narrative)
metadata:
  type: project
---

**Standing call (in force rest of project, from 2026-08-05 external review): DO NOT ADD SCOPE. GUARD THE
EXITS.** Scope is already at the outer edge of what one solo dev finishes in the ~4-5 weeks left. Nothing
gets added without cutting something in the same breath. Every `/standup` is measured against protecting
the Week-7 demo + dashboard exit, not feature count.

**Why:** The review is blunt that the failure mode is slippage-into-no-demo, not under-scoping. Guarding
the exits (a shippable demo video + light dashboard) beats adding features.

**How to apply:** When any role proposes new work, first question stays "does this need to exist for v1?"
If yes, name what gets cut to make room. Two specific cuts already recorded (in ROADMAP.md cut/deferred log):

- **(a) No YOLOv8 bolt-on for resume keywords.** The classical-CV blob baseline cleared the safety bar;
  a learned model must beat it on the same eval harness — none has. Adding YOLO speculatively for keywords
  loses the metric-driven story. ONLY legitimate opening: a learned detector that beats the blob baseline
  on the REAL render, where precision was 0.445 — an honest before/after, and only then.
- **(b) No retrofitted startup / "billion-dollar" narrative.** Sim-only solo portfolio project built to
  get hired. Inflating it into a business pitch converts the project's best asset (ADR honesty) into an
  interview red flag. Honest framing already at ROADMAP.md ~line 34 stands; don't re-argue it.

See [[phase]].

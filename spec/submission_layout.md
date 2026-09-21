# Submission layout — how to package `submission.tar.gz` correctly

Not required by the grader as a file, and not part of the graded runtime — a standing reference so
we stop re-discovering this the hard way. Applies to every EECS 367 (autorob.org) project, not
just this one.

## The rule

**Every project's spec requires**: `submission.tar.gz` must extract to one project root whose
*direct* contents include that submission's `Makefile` — i.e. `Makefile` at the archive root, not
nested inside a wrapping directory.

**The mistake that breaks this** (hit on both Project 1 and Project 2): building the archive with

```bash
tar czf submission.tar.gz --exclude=... .
```

— tarring `.` (the current directory) — looks like it should produce a flat archive, but GNU tar
includes an explicit `.` directory member in the archive for this invocation. The grader's
archive-safety check (`grader/setup_submission.py`, `_safe_parts()`) rejects that `.` member
outright as an unsafe path, and extraction fails before anything else runs:

```
Traceback (most recent call last):
  File "/home/autograder/working_dir/grader/setup_submission.py", line 101, in <module>
    raise SystemExit(main())
  File "/home/autograder/working_dir/grader/setup_submission.py", line 96, in main
    extract_submission(args.archive, args.destination)
  File "/home/autograder/working_dir/grader/setup_submission.py", line 41, in extract_submission
    normalized = [(member, _safe_parts(member.name)) for member in members]
  File "/home/autograder/working_dir/grader/setup_submission.py", line 27, in _safe_parts
    raise UnsafeArchive(f"unsafe archive path: {name!r}")
UnsafeArchive: unsafe archive path: '.'
```

This is confirmed independently by Project 2's own `make_submission.sh`, which explicitly works
around it in a comment: *"List top-level entries by name (rather than tarring '.'): GNU tar
includes an explicit '.' directory member for `tar -C dir -czf out .`, which the grader's
archive-safety check ... rejects outright as an unsafe path. Named entries avoid that member
entirely."*

## The fix

**List the top-level entries by name instead of tarring `.`.** From the project root:

```bash
tar czf submission.tar.gz \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='.DS_Store' \
  --exclude='build' --exclude='bin' --exclude='.build' --exclude='target' \
  Makefile README.md .gitignore src tests <other top-level dirs/files that belong in the submission>
```

i.e. `tar czf OUTPUT [--exclude=...] ENTRY1 ENTRY2 ENTRY3 ...` — never a bare `.` as the thing
being archived. If a project's student kit ships a `make_submission.sh`, prefer running that (it
already does this correctly); otherwise apply this pattern by hand.

## What to leave out of the submission

- Generated/cache artifacts: `__pycache__/`, `*.pyc`, `.DS_Store`, and (for compiled languages)
  `build/`, `bin/`, `.build/`, `target/`.
- This repo's `prompts/`/`agent-notes/` workflow scaffolding — not part of the graded runtime.
  (The spec allows extra files in a submission, so including them wouldn't break anything, but
  we've been leaving them out for a clean, minimal archive.)
- The archive itself, if re-running this in the same repo (avoid `submission.tar.gz` including a
  stale copy of itself) — already covered by `.gitignore`.

## Verifying before you upload

```bash
tar tzf submission.tar.gz | sort          # eyeball the entry list -- no bare "." entry
tar tzf submission.tar.gz | grep -qx 'Makefile' && echo "Makefile at archive root: OK"
```

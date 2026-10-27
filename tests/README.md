# Tests

1. `make test` runs `python3 tests/run_tests.py`, which discovers and runs every `tests/test_*.py`.
2. Each `tests/test_*.py` also runs on its own, e.g. `python3 tests/test_integrators.py`.
3. Tests that start their own server pick a free port, never 9095, so they pass while `make run` is up.

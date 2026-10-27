"""Run every tests/test_*.py; this is what `make test` invokes (tests/README.md, rule 1)."""
import os
import sys
import unittest

suite = unittest.defaultTestLoader.discover(os.path.dirname(os.path.abspath(__file__)))
result = unittest.TextTestRunner(verbosity=2).run(suite)
sys.exit(0 if result.wasSuccessful() else 1)

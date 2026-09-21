import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import expr  # noqa: E402


def _eval(source: str, t: float) -> float:
    return expr.parse_function(source)(t)


class TestValidExpressions(unittest.TestCase):
    def test_bare_t(self):
        self.assertAlmostEqual(_eval("t", 3.5), 3.5)

    def test_sin(self):
        self.assertAlmostEqual(_eval("sin(t)", math.pi / 2), 1.0)

    def test_polynomial(self):
        # t^2 + 3*t - 1 at t=2 -> 4 + 6 - 1 = 9
        self.assertAlmostEqual(_eval("t^2 + 3*t - 1", 2.0), 9.0)

    def test_leading_unary_minus(self):
        # -t + 1 at t=2 -> -2 + 1 = -1
        self.assertAlmostEqual(_eval("-t + 1", 2.0), -1.0)

    def test_precedence_additive_vs_multiplicative(self):
        self.assertAlmostEqual(_eval("2 + 3*4", 0.0), 14.0)

    def test_precedence_power_over_multiplicative(self):
        self.assertAlmostEqual(_eval("2*3^2", 0.0), 18.0)

    def test_power_right_associative(self):
        # 2^3^2 = 2^(3^2) = 2^9 = 512, not (2^3)^2 = 64
        self.assertAlmostEqual(_eval("2^3^2", 0.0), 512.0)

    def test_unary_minus_binds_tighter_than_power(self):
        # per spec precedence (unary minus higher than ^): -2^2 = (-2)^2 = 4
        self.assertAlmostEqual(_eval("-2^2", 0.0), 4.0)

    def test_parentheses_override_precedence(self):
        self.assertAlmostEqual(_eval("(2+3)*4", 0.0), 20.0)

    def test_division(self):
        self.assertAlmostEqual(_eval("t/4", 10.0), 2.5)

    def test_named_functions(self):
        self.assertAlmostEqual(_eval("cos(0)", 0.0), 1.0)
        self.assertAlmostEqual(_eval("exp(0)", 0.0), 1.0)
        self.assertAlmostEqual(_eval("sqrt(4)", 0.0), 2.0)
        self.assertAlmostEqual(_eval("ln(1)", 0.0), 0.0)
        self.assertAlmostEqual(_eval("abs(-5)", 0.0), 5.0)
        self.assertAlmostEqual(_eval("tan(0)", 0.0), 0.0)

    def test_nested_function_and_expression(self):
        self.assertAlmostEqual(_eval("sin(t^2)", math.sqrt(math.pi / 2)), 1.0)

    def test_decimal_literals(self):
        self.assertAlmostEqual(_eval(".5 + 1.", 0.0), 1.5)

    def test_whitespace_is_insignificant(self):
        self.assertAlmostEqual(_eval("  t  +  1  ", 2.0), 3.0)


class TestMalformedExpressions(unittest.TestCase):
    def test_empty_string(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("")

    def test_unbalanced_open_paren(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("(1+2")

    def test_unbalanced_close_paren(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("1+2)")

    def test_unknown_identifier(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("foo(t)")

    def test_trailing_garbage(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("1 + 1 1")

    def test_adjacent_tokens_no_operator(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("1 2")

    def test_unrecognized_character(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("t & 1")

    def test_dangling_operator(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("1 +")

    def test_function_missing_parens(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function("sin t")

    def test_non_string_input(self):
        with self.assertRaises(expr.ExpressionError):
            expr.parse_function(123)  # type: ignore[arg-type]


class TestDomainErrorsBecomeNanInf(unittest.TestCase):
    """Domain errors at evaluation time must not raise -- they should behave
    like ordinary floating-point arithmetic (NaN/inf), per spec."""

    def test_sqrt_of_negative_is_nan(self):
        result = _eval("sqrt(-1)", 0.0)
        self.assertTrue(math.isnan(result))

    def test_ln_of_negative_is_nan(self):
        result = _eval("ln(-1)", 0.0)
        self.assertTrue(math.isnan(result))

    def test_ln_of_zero_is_negative_infinity(self):
        result = _eval("ln(0)", 0.0)
        self.assertEqual(result, float("-inf"))

    def test_division_by_zero_does_not_raise(self):
        # 1/t at t=0 -- Python raises ZeroDivisionError for float division by
        # zero, which is NOT the same as IEEE-754 inf; this is intentionally
        # exercised here since it's an easy thing to get wrong.
        try:
            result = _eval("1/t", 0.0)
        except ZeroDivisionError:
            self.fail("division by zero raised instead of producing inf")
        self.assertTrue(math.isinf(result))

    def test_negative_base_fractional_exponent_is_nan_not_complex(self):
        # (-4)^0.5 -- Python's ** silently returns a complex number here,
        # which must not leak out of this module (it isn't JSON-serializable
        # and previously caused the whole request to hang downstream).
        try:
            result = _eval("(-4)^0.5", 0.0)
        except Exception as exc:  # noqa: BLE001
            self.fail(f"raised {exc!r} instead of producing NaN")
        self.assertIsInstance(result, float)
        self.assertTrue(math.isnan(result))

    def test_zero_to_negative_power_is_infinity(self):
        # 0^-1 -- Python raises ZeroDivisionError for this.
        try:
            result = _eval("0^-1", 0.0)
        except ZeroDivisionError:
            self.fail("0^-1 raised instead of producing inf")
        self.assertTrue(math.isinf(result))

    def test_power_overflow_is_infinity_not_raised(self):
        # t^1000 at t=500 -- Python raises OverflowError instead of inf.
        try:
            result = _eval("t^1000", 500.0)
        except OverflowError:
            self.fail("t^1000 raised OverflowError instead of producing inf")
        self.assertTrue(math.isinf(result))

    def test_trig_of_infinite_input_is_nan(self):
        # sin(1/t) at t=0 -- 1/t is inf, and math.sin(inf) raises ValueError.
        try:
            result = _eval("sin(1/t)", 0.0)
        except ValueError:
            self.fail("sin(inf) raised instead of producing NaN")
        self.assertTrue(math.isnan(result))


if __name__ == "__main__":
    unittest.main()

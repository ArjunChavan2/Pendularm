"""Small math-expression parser/evaluator for /arm_sim/integration_step's
`function` field (spec/PROJECT2_PENDULARM.md, "The `function` field").

Grammar (precedence lowest to highest, per spec): +/- , then */ , then ^
(right-associative), then unary minus, then atoms (numbers, `t`, named
single-argument functions, parenthesized expressions).

This module only turns a string into a `Callable[[float], float]` (a
function of `t`); it has no knowledge of the rest of the runtime.
"""
from __future__ import annotations

import math
import re
from typing import Callable


class ExpressionError(ValueError):
    """Raised for a `function` string that fails to parse. Callers (the
    /arm_sim/integration_step service handler) should catch this and reject
    the request with result:false, not let it propagate."""


def _safe_sqrt(x: float) -> float:
    return float("nan") if x < 0 else math.sqrt(x)


def _safe_ln(x: float) -> float:
    if x < 0:
        return float("nan")
    if x == 0:
        return float("-inf")
    return math.log(x)


def _safe_exp(x: float) -> float:
    try:
        return math.exp(x)
    except OverflowError:
        return float("inf") if x > 0 else 0.0


def _safe_div(a: float, b: float) -> float:
    # Python raises ZeroDivisionError for float division by zero, unlike
    # IEEE-754 (which the spec's "let it become NaN/inf" language implies).
    if b != 0:
        return a / b
    if a == 0:
        return float("nan")
    sign = math.copysign(1.0, a) * math.copysign(1.0, b)
    return float("inf") if sign > 0 else float("-inf")


def _safe_pow(base: float, exponent: float) -> float:
    # Python's ** raises ZeroDivisionError for 0**negative and OverflowError
    # for a result too large to represent, and silently returns a `complex`
    # for (negative base)**(non-integer exponent) -- none of which match
    # IEEE-754/"ordinary floating-point arithmetic" (NaN/inf, never raise,
    # never a non-float type a JSON response can't carry).
    try:
        result = base ** exponent
    except ZeroDivisionError:  # base == 0, exponent < 0
        return float("inf")
    except OverflowError:
        if base >= 0:
            return float("inf")
        # negative base: sign depends on whether the exponent is an odd integer
        if exponent == int(exponent) and int(exponent) % 2 != 0:
            return float("-inf")
        return float("inf")
    if isinstance(result, complex):
        return float("nan")
    return result


def _safe_trig(fn: Callable[[float], float]) -> Callable[[float], float]:
    # math.sin/cos/tan raise ValueError for non-finite input (e.g. sin(inf)).
    def wrapped(x: float) -> float:
        return float("nan") if not math.isfinite(x) else fn(x)

    return wrapped


_FUNCTIONS: dict[str, Callable[[float], float]] = {
    "sin": _safe_trig(math.sin),
    "cos": _safe_trig(math.cos),
    "tan": _safe_trig(math.tan),
    "exp": _safe_exp,
    "sqrt": _safe_sqrt,
    "ln": _safe_ln,
    "abs": abs,
}

_TOKEN_PATTERN = re.compile(
    r"""
      (?P<WS>\s+)
    | (?P<NUMBER>\d+\.\d+|\.\d+|\d+\.|\d+)
    | (?P<IDENT>[A-Za-z_][A-Za-z_0-9]*)
    | (?P<OP>[+\-*/^()])
    """,
    re.VERBOSE,
)


def _tokenize(source: str) -> list[tuple[str, str]]:
    tokens: list[tuple[str, str]] = []
    pos, length = 0, len(source)
    while pos < length:
        m = _TOKEN_PATTERN.match(source, pos)
        if not m:
            raise ExpressionError(f"unrecognized character {source[pos]!r} at position {pos}")
        kind = m.lastgroup
        text = m.group()
        if kind != "WS":
            tokens.append((kind, text))
        pos = m.end()
    tokens.append(("END", ""))
    return tokens


class _Parser:
    def __init__(self, tokens: list[tuple[str, str]]) -> None:
        self._tokens = tokens
        self._pos = 0

    def _peek(self) -> tuple[str, str]:
        return self._tokens[self._pos]

    def _advance(self) -> tuple[str, str]:
        tok = self._tokens[self._pos]
        self._pos += 1
        return tok

    def _expect_op(self, text: str) -> None:
        kind, value = self._peek()
        if kind != "OP" or value != text:
            raise ExpressionError(f"expected {text!r}, found {value!r}")
        self._advance()

    def parse(self) -> Callable[[float], float]:
        result = self._parse_expr()
        kind, value = self._peek()
        if kind != "END":
            raise ExpressionError(f"unexpected trailing input: {value!r}")
        return result

    def _parse_expr(self) -> Callable[[float], float]:
        left = self._parse_term()
        while True:
            kind, value = self._peek()
            if kind == "OP" and value in ("+", "-"):
                self._advance()
                right = self._parse_term()
                left = self._combine(left, right, value)
            else:
                return left

    def _parse_term(self) -> Callable[[float], float]:
        left = self._parse_power()
        while True:
            kind, value = self._peek()
            if kind == "OP" and value in ("*", "/"):
                self._advance()
                right = self._parse_power()
                left = self._combine(left, right, value)
            else:
                return left

    def _parse_power(self) -> Callable[[float], float]:
        base = self._parse_unary()
        kind, value = self._peek()
        if kind == "OP" and value == "^":
            self._advance()
            exponent = self._parse_power()  # right-associative
            return self._combine(base, exponent, "^")
        return base

    def _parse_unary(self) -> Callable[[float], float]:
        kind, value = self._peek()
        if kind == "OP" and value == "-":
            self._advance()
            operand = self._parse_unary()
            return (lambda o: (lambda t: -o(t)))(operand)
        return self._parse_atom()

    def _parse_atom(self) -> Callable[[float], float]:
        kind, value = self._peek()
        if kind == "NUMBER":
            self._advance()
            num = float(value)
            return lambda t: num
        if kind == "IDENT":
            self._advance()
            if value == "t":
                return lambda t: t
            if value in _FUNCTIONS:
                self._expect_op("(")
                arg = self._parse_expr()
                self._expect_op(")")
                fn = _FUNCTIONS[value]
                return (lambda f, a: (lambda t: f(a(t))))(fn, arg)
            raise ExpressionError(f"unknown identifier {value!r}")
        if kind == "OP" and value == "(":
            self._advance()
            inner = self._parse_expr()
            self._expect_op(")")
            return inner
        raise ExpressionError(f"unexpected token {value!r}" if value else "unexpected end of input")

    @staticmethod
    def _combine(left: Callable[[float], float], right: Callable[[float], float], op: str) -> Callable[[float], float]:
        if op == "+":
            return lambda t: left(t) + right(t)
        if op == "-":
            return lambda t: left(t) - right(t)
        if op == "*":
            return lambda t: left(t) * right(t)
        if op == "/":
            return lambda t: _safe_div(left(t), right(t))
        if op == "^":
            return lambda t: _safe_pow(left(t), right(t))
        raise AssertionError(f"unreachable operator {op!r}")


def parse_function(source: str) -> Callable[[float], float]:
    """Parse a `function` field string into a callable f(t) -> float.

    Raises ExpressionError (a ValueError subclass) for anything that fails to
    parse cleanly: empty input, unbalanced parentheses, an unknown
    identifier, trailing garbage, adjacent tokens with no operator between
    them, or an unrecognized character. Domain errors at evaluation time
    (e.g. sqrt of a negative number) are not raised here -- they surface as
    NaN/inf when the returned callable is actually invoked, same as ordinary
    floating-point arithmetic.
    """
    if not isinstance(source, str):
        raise ExpressionError(f"function must be a string, got {type(source).__name__}")
    return _Parser(_tokenize(source)).parse()

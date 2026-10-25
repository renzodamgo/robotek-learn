#!/usr/bin/env python3
"""Validate reto/questions.json (the Kahoot-style question bank). Pure Python, no dependencies.

Usage: python3 reto/validate_questions.py [path/to/questions.json]
Exit code 0 when there are no errors (warnings are only printed), 1 otherwise.
"""
import json
import re
import sys
from collections import Counter
from pathlib import Path

STEPS, PER_STEP, OPTIONS = 9, 5, 4
MAX_Q, MAX_OPTION, MAX_WHY = 140, 60, 160
WARN_WORDS, ERROR_WORDS = 20, 24      # words per sentence
BAD_CHARS = {"\u2014": "em dash", "\u2013": "en dash", "\u2212": "minus sign (use -)"}
NEGATION = re.compile(r"\b(excepto|salvo)\b|\bNO\b|\bNOT\b")

errors, warnings = [], []


def err(where, msg):
    errors.append(f"{where}: {msg}")


def warn(where, msg):
    warnings.append(f"{where}: {msg}")


def sentences(text):
    return [s.strip() for s in re.split(r"(?<=[.?!])\s+|[¿¡]", text) if s.strip()]


def check_text(where, field, text, limit):
    if not isinstance(text, str) or not text.strip():
        err(where, f"{field} must be a non-empty string")
        return
    if len(text) > limit:
        err(where, f"{field} has {len(text)} chars (max {limit})")
    for ch, name in BAD_CHARS.items():
        if ch in text:
            err(where, f"{field} contains a {name}")
    if text != text.strip() or "  " in text:
        err(where, f"{field} has stray spaces")


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("questions.json")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:                       # noqa: BLE001 - report any read or parse failure
        print(f"ERROR cannot read {path}: {exc}")
        return 1
    if data.get("version") != 1:
        err("root", "version must be 1")
    steps = data.get("steps")
    if not isinstance(steps, list) or len(steps) != STEPS:
        err("root", f"steps must be a list of {STEPS}")
        steps = steps if isinstance(steps, list) else []

    overall, longest_correct, ratios = Counter(), 0, []
    all_q = []
    for index, step in enumerate(steps, start=1):
        sw = f"step {index}"
        if step.get("step") != index:
            err(sw, f"'step' must be {index}, got {step.get('step')!r}")
        check_text(sw, "title", step.get("title"), 40)
        questions = step.get("questions")
        if not isinstance(questions, list) or len(questions) != PER_STEP:
            err(sw, f"questions must be a list of {PER_STEP}")
            continue
        answers = []
        for number, item in enumerate(questions, start=1):
            where = f"S{index}Q{number}"
            extra = set(item) - {"q", "options", "answer", "why", "ref"}
            missing = {"q", "options", "answer", "why", "ref"} - set(item)
            if extra or missing:
                err(where, f"keys: missing {sorted(missing)}, unexpected {sorted(extra)}")
                continue
            check_text(where, "q", item["q"], MAX_Q)
            check_text(where, "why", item["why"], MAX_WHY)
            check_text(where, "ref", item["ref"], 120)
            all_q.append(item["q"])
            options = item["options"]
            if not isinstance(options, list) or len(options) != OPTIONS:
                err(where, f"options must be a list of {OPTIONS}")
                continue
            for k, option in enumerate(options):
                check_text(where, f"option {k}", option, MAX_OPTION)
            if len(set(options)) != OPTIONS:
                err(where, "options must be distinct")
            answer = item["answer"]
            if not isinstance(answer, int) or isinstance(answer, bool) or not 0 <= answer < OPTIONS:
                err(where, f"answer must be an int 0..{OPTIONS - 1}")
                continue
            answers.append(answer)
            overall[answer] += 1
            lengths = [len(o) for o in options]
            if lengths[answer] == max(lengths) and lengths.count(max(lengths)) == 1:
                err(where, "the correct option is the longest one")
                longest_correct += 1
            if max(lengths) > 3 * min(lengths):
                warn(where, f"option lengths differ a lot ({min(lengths)} vs {max(lengths)} chars)")
            ratios.append(lengths[answer] / (sum(lengths) / OPTIONS))
            for field in ("q", "why"):
                for sentence in sentences(item[field]):
                    words = len(sentence.split())
                    if words > ERROR_WORDS:
                        err(where, f"{field}: sentence of {words} words: {sentence[:50]}...")
                    elif words > WARN_WORDS:
                        warn(where, f"{field}: sentence of {words} words: {sentence[:50]}...")
            if NEGATION.search(item["q"]):
                warn(where, "the question looks like a negation (avoid)")
        for a in range(len(answers) - 2):
            if answers[a] == answers[a + 1] == answers[a + 2]:
                err(sw, f"answer index {answers[a]} three times in a row (questions {a + 1}-{a + 3})")
    if len(set(all_q)) != len(all_q):
        err("root", "duplicated question text")
    total = sum(overall.values())
    if total == STEPS * PER_STEP:
        expected = total / OPTIONS
        for k in range(OPTIONS):
            if abs(overall[k] - expected) > 2:
                err("root", f"answer index {k} appears {overall[k]} times (expected about {expected:.0f})")

    print(f"file: {path}")
    print(f"steps: {len(steps)}   questions: {total}")
    for step in steps:
        print(f"  step {step.get('step')}: {step.get('title')!r:24} answers {[q.get('answer') for q in step.get('questions', [])]}")
    print("answer index counts: " + ", ".join(f"{k}: {overall[k]}" for k in range(OPTIONS)))
    if ratios:
        print(f"correct option length / mean option length: avg {sum(ratios) / len(ratios):.2f}, "
              f"max {max(ratios):.2f}; correct is the longest in {longest_correct} questions")
    for w in warnings:
        print("WARN  " + w)
    for e in errors:
        print("ERROR " + e)
    print(f"{len(errors)} errors, {len(warnings)} warnings -> {'FAIL' if errors else 'OK'}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())

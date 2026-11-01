#!/usr/bin/env python3
"""Write reto/kahoot-preguntas.md from reto/questions.json: every question with its options, the correct one marked.

Usage: python3 reto/questions_to_md.py [questions.json] [output.md]
Run it again after editing questions.json so both files say the same.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
LETTERS = "ABCD"


def render(data):
    total = sum(len(step["questions"]) for step in data["steps"])
    out = [
        "# Preguntas del Reto (el Kahoot del curso)",
        "",
        f"{total} preguntas en {len(data['steps'])} pasos de la guía del Open. La respuesta correcta va en negrita "
        "y con ✅. Cada pregunta trae la explicación que se muestra al revelar y dónde repasarlo.",
        "",
        "Este archivo se genera desde `reto/questions.json` con `python3 reto/questions_to_md.py`. "
        "No lo edites a mano: cambia el JSON, pasa `python3 reto/validate_questions.py` y vuelve a generarlo.",
        "",
    ]
    for step in data["steps"]:
        out += [f"## Paso {step['step']}. {step['title']}", ""]
        for number, item in enumerate(step["questions"], start=1):
            out += [f"### {step['step']}.{number}. {item['q']}", ""]
            for k, option in enumerate(item["options"]):
                if k == item["answer"]:
                    out.append(f"- **{LETTERS[k]}. {option}** ✅")
                else:
                    out.append(f"- {LETTERS[k]}. {option}")
            out += ["", f"Por qué: {item['why']}", "", f"Repasar: {item['ref']}", ""]
    return "\n".join(out).rstrip("\n") + "\n"


def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "questions.json"
    target = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "kahoot-preguntas.md"
    target.write_text(render(json.loads(source.read_text(encoding="utf-8"))), encoding="utf-8")
    print(f"wrote {target}")


if __name__ == "__main__":
    main()

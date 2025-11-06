"""
Example usage
-------------
`result = nl_to_project_code("Write a new example query based on the findings in the files.")`
"""

DOC_REFERENCE_PATH = ["/home/tura/workspace/src/krrood/examples/eql/*.md"]

CODE_SCHEMA = {
    "type": "object",
    "properties": {
        "filename": {"type": "string"},
        "summary": {"type": "string"},
        "code": {"type": "string"},
        "tests": {"type": "string"},
        "dependencies": {
            "type": "array",
            "items": {"type": "string"}
        },
        "notes": {"type": "string"}
    },
    "required": ["filename", "code"],
    "additionalProperties": False
}

SYSTEM_PROMPT = """You write Python for THIS project only.

Project rules:
- Use our internal APIs only. If unsure, ask to call tools to read docs.
- Style: PEP8, type hints, docstrings.
- Include minimal tests targeting pytest.
- If request is risky or ambiguous, return a safe minimal stub.

Output only as per the provided JSON schema. Do not wrap in markdown fences."""

TOOLS = [{
    "type": "function",
    "function": {
        "name": "read_files",
        "description": "Read project files by glob patterns (e.g., 'src/**/*.py', docs/*.md').",
        "parameters": {
            "type": "object",
            "properties": {
                "patterns": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": f"Glob patterns of files to read."
                },
                "max_chars": {"type": "integer", "minimum": 1000, "maximum": 200000}
            },
            "required": ["patterns"]
        }
    }
}]


from typing import Dict, Any, List
from jsonschema import validate
import ollama
import json
import re
import os
from glob import glob

# --- (A) Tooling: allow the model to read project docs/APIs ---

def read_project_files(patterns: List[str], max_chars: int = 20_000) -> str:
    """Return concatenated snippets of files matching given glob patterns."""
    paths = []
    for p in patterns:
        paths.extend(glob(p, recursive=True))
    chunks = []
    total = 0
    for path in sorted(set(paths)):
        if not os.path.isfile(path):
            continue
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
        header = f"\n--- FILE: {path} ---\n"
        take = max_chars - total - len(header)
        if take <= 0:
            break
        snippet = text[: take]
        chunks.append(header + snippet)
        total += len(header) + len(snippet)
    return "".join(chunks) if chunks else "(no files matched)"

AVAILABLE_FUNCTIONS = {"read_files": read_project_files}


def nl_to_project_code(
    request: str,
    model: str = "gpt-oss:20b",
    doc_globs: List[str] = DOC_REFERENCE_PATH,
) -> Dict[str, Any]:
    client = ollama.Client(host="http://localhost:11434")  # defaults to http://localhost:11434

    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": (
            "Task: Convert this natural-language request into Python code for this project.\n"
            "You may call tools to inspect project files before coding.\n\n"
            "The response should be in the given code schema."
            " Respond with the code explanation into the 'summary' field and"
            " the raw code segments all together in the 'code' field."
            " Do not put explanations in the 'code' segment.\n\n"
            f"User request:\n{request}\n\n"
            f"Look into files {doc_globs}\n"
        )},
    ]

    while True:
        resp = client.chat(
            model=model,
            messages=messages,
            tools=TOOLS,            # enable tool calling
            format=CODE_SCHEMA,     # structured output (JSON schema)
            stream=False
        )

        # If the model asked to call a tool, execute and loop
        if "tool_calls" in resp.get("message", {}):
            for call in resp["message"]["tool_calls"]:
                name = call["function"]["name"]
                args = call["function"].get("arguments", {})
                fn = AVAILABLE_FUNCTIONS.get(name)
                print(f"(calling tool '{name}' with args {args})")
                if fn is None:
                    tool_result = f"(tool '{name}' not available)"
                else:
                    try:
                        tool_result = fn(**args)
                    except TypeError:
                        tool_result = f"(bad arguments for tool '{name}': {args})"
                messages.append({
                    "role": "tool",
                    "name": name,
                    "content": tool_result
                })
            # loop back so the model can use the tool results
            continue

        raw = resp["message"]["content"]
        print(str(raw))

        # sometimes models sneak in markdown fences—strip them defensively
        cleaned = re.sub(r"^\s*```(?:json)?|```\s*$", "", raw.strip(), flags=re.IGNORECASE | re.MULTILINE)
        try:
            data = json.loads(cleaned)
            validate(instance=data, schema=CODE_SCHEMA)
            return data
        except json.JSONDecodeError as e:
            # Handle JSON decoding errors - log the error and return a safe fallback
            print(f"JSONDecodeError: {e}")  # Consider logging this
            return "(no valid JSON found)"  # Return a safe fallback
        except Exception as e:
            # Handle other exceptions - log the error and return a safe fallback
            print(f"Other error: {e}")  # Consider logging this
            # Ask the model to fix its own output
            messages.append({
                "role": "assistant",
                "content": raw
            })
            messages.append({
                "role": "user",
                "content": (
                    "Your output did not match the JSON schema. "
                    "Return ONLY valid JSON per the schema, no markdown, no commentary."
                )
            })
            return "(no valid JSON found)"


if __name__ == "__main__":
    result = nl_to_project_code(
        request = f"Write a new example query based on the findings in the files.",
        doc_globs = DOC_REFERENCE_PATH
    )
    print(json.dumps(result, indent=2))

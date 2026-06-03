#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""project3/test 내 .py/.ino 파일에서 주석만 제거. docstring/shebang/coding 선언은 유지."""

import io
import re
import sys
import tokenize
from pathlib import Path

ROOT = Path(__file__).resolve().parent / "project3" / "test"
CODING_RE = re.compile(r"coding[:=]\s*[-\w.]+")


def keep_comment(lineno, text):
    """shebang(첫 줄 #!)과 인코딩 선언(1~2번째 줄)만 보존."""
    if lineno == 1 and text.startswith("#!"):
        return True
    if lineno <= 2 and CODING_RE.search(text):
        return True
    return False


def strip_py(source):
    lines = source.splitlines(keepends=True)
    # 라인별 (시작 col, 보존여부) - 파이썬 주석은 한 줄에 최대 1개
    comments = {}
    reader = io.StringIO(source).readline
    for tok in tokenize.generate_tokens(reader):
        if tok.type == tokenize.COMMENT:
            row = tok.start[0]
            col = tok.start[1]
            comments[row] = (col, keep_comment(row, tok.string))

    out = []
    for idx, raw in enumerate(lines, start=1):
        if idx in comments and not comments[idx][1]:
            col = comments[idx][0]
            head = raw[:col]
            newline = "\n" if raw.endswith("\n") else ""
            if head.strip() == "":
                # 주석만 있던 줄 → 통째로 삭제
                continue
            out.append(head.rstrip() + newline)
        else:
            out.append(raw)
    return "".join(out)


def strip_cpp(source):
    """C/C++(.ino) // 와 /* */ 주석 제거. 문자열/문자 리터럴 보존."""
    res = []
    i = 0
    n = len(source)
    state = "code"  # code, line_comment, block_comment, string, char
    while i < n:
        c = source[i]
        nxt = source[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                state = "line_comment"
                i += 2
                continue
            if c == "/" and nxt == "*":
                state = "block_comment"
                i += 2
                continue
            if c == '"':
                state = "string"
                res.append(c)
                i += 1
                continue
            if c == "'":
                state = "char"
                res.append(c)
                i += 1
                continue
            res.append(c)
            i += 1
        elif state == "string":
            res.append(c)
            if c == "\\" and i + 1 < n:
                res.append(source[i + 1])
                i += 2
                continue
            if c == '"':
                state = "code"
            i += 1
        elif state == "char":
            res.append(c)
            if c == "\\" and i + 1 < n:
                res.append(source[i + 1])
                i += 2
                continue
            if c == "'":
                state = "code"
            i += 1
        elif state == "line_comment":
            if c == "\n":
                res.append(c)
                state = "code"
            i += 1
        elif state == "block_comment":
            if c == "*" and nxt == "/":
                state = "code"
                i += 2
                continue
            # 블록 주석 내부 개행은 줄 구조 유지를 위해 보존
            if c == "\n":
                res.append(c)
            i += 1

    stripped = "".join(res)
    # 주석 제거로 비게 된 줄만 삭제 (원래 빈 줄은 보존)
    src_lines = source.splitlines()
    new_lines = stripped.splitlines()
    out = []
    for orig, new in zip(src_lines, new_lines):
        if orig.strip() != "" and new.strip() == "":
            continue
        out.append(new.rstrip())
    # 길이 불일치(블록주석 다중행 등) 대비: 남은 줄 처리
    if len(new_lines) > len(src_lines):
        for new in new_lines[len(src_lines):]:
            if new.strip() != "":
                out.append(new.rstrip())
    trailing = "\n" if source.endswith("\n") else ""
    return "\n".join(out) + trailing


def main():
    py_files = sorted(ROOT.rglob("*.py"))
    ino_files = sorted(ROOT.rglob("*.ino"))
    changed = []

    for path in py_files:
        text = path.read_text(encoding="utf-8")
        new = strip_py(text)
        if new != text:
            path.write_text(new, encoding="utf-8")
            changed.append(path)

    for path in ino_files:
        text = path.read_text(encoding="utf-8")
        new = strip_cpp(text)
        if new != text:
            path.write_text(new, encoding="utf-8")
            changed.append(path)

    print(f"processed py={len(py_files)} ino={len(ino_files)} changed={len(changed)}")
    for path in changed:
        print(" -", path.relative_to(ROOT))


if __name__ == "__main__":
    sys.exit(main())

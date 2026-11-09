#!/usr/bin/env python3
"""Talk to the MCP for Blender addon directly, bypassing the MCP server.

  python3 scripts/blender_doctor.py              # health check: version + scene summary
  python3 scripts/blender_doctor.py file.py      # run a Blender Python file in the live session
  python3 scripts/blender_doctor.py file.py --args '{"k": 1}'   # file sees ARGS = {"k": 1}
  echo "print(1)" | python3 scripts/blender_doctor.py -

Useful for diagnosing the connection or scripting Blender from outside an agent session.
"""
import json
import os
import socket
import sys

HOST = os.environ.get("BLENDER_HOST", "localhost")
PORT = int(os.environ.get("BLENDER_PORT", "9877"))   # 9876 is taken by the trainnr Studio viewer


def send(cmd_type, params=None, timeout=180.0):
    with socket.create_connection((HOST, PORT), timeout=5) as sock:
        sock.sendall(json.dumps({"type": cmd_type, "params": params or {}}).encode())
        sock.settimeout(timeout)
        buf = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
            try:
                resp = json.loads(buf)
                break
            except json.JSONDecodeError:
                continue
    if resp.get("status") == "error":
        raise RuntimeError(resp.get("message", "unknown error from Blender"))
    return resp.get("result", {})


def main():
    if len(sys.argv) < 2:
        code = (
            "import bpy\n"
            "print('Blender', bpy.app.version_string)\n"
            "for o in bpy.context.scene.objects:\n"
            "    print(f'  {o.name:24} {o.type:8} loc={tuple(round(v, 4) for v in o.location)}')\n"
        )
    else:
        code = sys.stdin.read() if sys.argv[1] == "-" else open(sys.argv[1]).read()
        if "--args" in sys.argv:  # JSON dict exposed to the script as ARGS
            args = json.loads(sys.argv[sys.argv.index("--args") + 1])
            code = f"ARGS = {args!r}\n" + code
    try:
        result = send("execute_code", {"code": code}, timeout=float(os.environ.get("BLENDER_TIMEOUT", "180")))
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        sys.exit(f"Cannot reach Blender addon at {HOST}:{PORT} ({e}). "
                 "Is Blender open with the MCP server started (N-panel → MCP for Blender)?")
    out = result.get("result", result) if isinstance(result, dict) else result
    print(out if isinstance(out, str) else json.dumps(out, indent=2))


if __name__ == "__main__":
    main()

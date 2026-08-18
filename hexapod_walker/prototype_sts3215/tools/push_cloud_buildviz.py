#!/usr/bin/env python3
"""Mirror this project's current DEFAULT BuildViz version to the CLOUD hub.

Standing convention (Lukas, Aug 2026): every local `verify-buildviz` publish
should also land on the CoreWeave-hosted BuildViz hub so the build is
viewable off this machine:

    https://buildviz.cwd1f0-new-cluster.coreweave.app/?project=prototype_sts3215&build=prototype_sts3215

This script reads the LOCAL hub cache (~/.buildviz/cache/<build-id>), takes
whatever version is currently the default (e.g. v64), and POSTs it to the
remote hub's /__buildviz/push with all mesh bytes attached (self-contained,
same payload the deploy/coreweave/migrate_local_builds.py migration uses)
plus the design spec, then marks it the remote default.

Auth: the remote hub requires X-API-Key.  The key lives in BUILDVIZ_API_KEY
(same key as the local hub; canonical source = the CoreWeave k8s secret
`buildviz-api-key`, readable via
    KUBECONFIG=~/.kube/coreweave.yaml kubectl get secret buildviz-api-key \
        -o jsonpath='{.data.key}' | base64 -d
).

Called from the Makefile at the end of `verify-buildviz` (failure-tolerant:
a dead network must not fail the local verify), or standalone:

    BUILDVIZ_API_KEY=... tools/push_cloud_buildviz.py [--build-id <id>]
"""

import argparse
import base64
import json
import os
import sys
import urllib.request
from pathlib import Path

DEFAULT_CLOUD_URL = "https://buildviz.cwd1f0-new-cluster.coreweave.app"
DEFAULT_BUILD_ID = "prototype_sts3215"
MAX_UPLOAD_BYTES = 256 * 1024 * 1024

BUILDVIZ_HOME = Path.home() / ".buildviz"
CACHE = BUILDVIZ_HOME / "cache"
ASSETS = CACHE / "_assets"


def _resolve_mesh_file(url: str, scene_dir: Path) -> Path | None:
    if url.startswith(("http://", "https://")):
        return None
    if url.startswith("/builds/_assets/"):
        candidate = ASSETS / Path(url).name
        return candidate if candidate.exists() else None
    if url.startswith("/"):
        return None
    candidate = scene_dir / url.lstrip("./")
    return candidate if candidate.exists() else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build-id", default=DEFAULT_BUILD_ID)
    ap.add_argument("--cloud-url",
                    default=os.environ.get("BUILDVIZ_CLOUD_URL",
                                           DEFAULT_CLOUD_URL))
    args = ap.parse_args()
    base = args.cloud_url.rstrip("/")

    api_key = os.environ.get("BUILDVIZ_API_KEY", "")
    if not api_key:
        print("push_cloud_buildviz: BUILDVIZ_API_KEY not set -- skipping "
              "cloud mirror (see docstring for the key source)",
              file=sys.stderr)
        return 1

    status = json.load(urllib.request.urlopen(
        f"{base}/__buildviz/status", timeout=30))
    if status.get("service") != "buildviz-hub":
        print(f"push_cloud_buildviz: {base} is not a BuildViz hub",
              file=sys.stderr)
        return 1

    build_dir = CACHE / args.build_id
    meta = json.loads((build_dir / "meta.json").read_text())
    version = meta.get("defaultVersion", "main")
    scene = json.loads((build_dir / "scene.json").read_text())

    assets = []
    for mesh in scene.get("meshes", []):
        url, mesh_id = mesh.get("url", ""), mesh.get("id", "")
        if not url or not mesh_id:
            continue
        file = _resolve_mesh_file(url, build_dir)
        if file is not None:
            assets.append({
                "meshId": mesh_id,
                "data": base64.b64encode(file.read_bytes()).decode(),
                "ext": file.suffix.lstrip(".") or "stl",
            })

    payload = {
        "buildId": args.build_id,
        "version": version,
        "scene": scene,
        "setDefault": True,
        "maxUploadBytes": MAX_UPLOAD_BYTES,
        "assets": assets,
    }
    name = meta.get("name")
    if name:
        payload["name"] = name
    spec = build_dir / "design_spec.yaml"
    if spec.exists():
        payload["designSpec"] = spec.read_text()

    req = urllib.request.Request(
        f"{base}/__buildviz/push",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "X-API-Key": api_key},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=300) as response:
        result = json.load(response)
    print(f"    cloud mirror: {result.get('summary', 'ok')} -> "
          f"{base}/?project={args.build_id.split('/')[0]}"
          f"&build={args.build_id.split('/')[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

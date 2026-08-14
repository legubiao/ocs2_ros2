#!/usr/bin/env python3
import json
import os
import re
import sys
import urllib.error
import urllib.request


def api_get(url, token):
    headers = {"Accept": "application/vnd.github+json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, headers=headers)
    try:
        with urllib.request.urlopen(req) as resp:
            return json.load(resp)
    except urllib.error.HTTPError:
        return None
    except Exception:
        return None


def parse_triplet(value):
    core = re.sub(r"^[vV]", "", value or "").split("-")[0]
    parts = core.split(".")
    while len(parts) < 3:
        parts.append("0")
    try:
        return int(parts[0]), int(parts[1]), int(parts[2])
    except ValueError as exc:
        raise SystemExit(f"Unable to parse semver from: {value}") from exc


def format_triplet(triple):
    return f"{triple[0]}.{triple[1]}.{triple[2]}"


def main():
    repo = os.environ["RELEASE_REPO"]
    short_sha = os.environ["SHORT_SHA"]
    fallback_xml = os.environ.get("FALLBACK_PACKAGE_XML", "")
    prerelease_tag = os.environ.get("PRE_RELEASE_TAG", "pre-release")
    token = os.environ.get("GH_TOKEN", "")

    latest = None
    latest_data = api_get(f"https://api.github.com/repos/{repo}/releases/latest", token)
    if latest_data and latest_data.get("tag_name"):
        latest = parse_triplet(latest_data["tag_name"])
        print(
            f"Latest formal release: {latest_data['tag_name']} -> {format_triplet(latest)}",
            file=sys.stderr,
        )
    elif fallback_xml and os.path.isfile(fallback_xml):
        with open(fallback_xml, encoding="utf-8") as fh:
            for line in fh:
                match = re.search(r"<version>([^<]+)</version>", line)
                if match:
                    latest = parse_triplet(match.group(1))
                    print(
                        f"No formal release on {repo}; using {fallback_xml} ({format_triplet(latest)}).",
                        file=sys.stderr,
                    )
                    break
        if latest is None:
            raise SystemExit(f"Unable to read version from {fallback_xml}.")

    if latest is None:
        latest = (0, 0, 0)
        print(f"No formal release on {repo}; starting from 0.0.1 base.", file=sys.stderr)

    # GitHub Release asset names store "~" as "." (e.g. 1.4.1.main.sha, not 1.4.1~main.sha).
    prerelease_base_re = re.compile(r"_(\d+\.\d+\.\d+)[~.]main\.")

    prev = None
    prev_name = ""
    pr_data = api_get(f"https://api.github.com/repos/{repo}/releases/tags/{prerelease_tag}", token)
    if pr_data:
        for asset in pr_data.get("assets", []):
            name = asset.get("name", "")
            match = prerelease_base_re.search(name)
            if not match:
                continue
            candidate = parse_triplet(match.group(1))
            if prev is None or candidate > prev:
                prev = candidate
                prev_name = name

        if prev is not None:
            print(
                f"Previous pre-release deb base from {prev_name}: {format_triplet(prev)}",
                file=sys.stderr,
            )

    if prev is None:
        nxt = (latest[0], latest[1], latest[2] + 1)
        print(
            f"No previous pre-release base; next = latest patch+1 -> {format_triplet(nxt)}",
            file=sys.stderr,
        )
    elif (prev[0], prev[1]) != (latest[0], latest[1]):
        nxt = (latest[0], latest[1], latest[2] + 1)
        print(
            f"Pre-release {prev[0]}.{prev[1]} != latest {latest[0]}.{latest[1]}; "
            f"reset to {format_triplet(nxt)}",
            file=sys.stderr,
        )
    else:
        nxt = (prev[0], prev[1], prev[2] + 1)
        print(
            f"Aligned with latest {latest[0]}.{latest[1]}; "
            f"bump {format_triplet(prev)} -> {format_triplet(nxt)}",
            file=sys.stderr,
        )

    print(f"{format_triplet(nxt)}~main.{short_sha}")


if __name__ == "__main__":
    main()

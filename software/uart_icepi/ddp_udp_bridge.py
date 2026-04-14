#!/usr/bin/env python3
import os
import sys


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    from ddp_udp_bridge import main as bridge_main

    bridge_main()


if __name__ == "__main__":
    main()

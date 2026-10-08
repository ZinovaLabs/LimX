"""
List, delete, restore and edit episodes recorded by vla/collect_vla_data.py.

Deleted episodes are moved to <folder>/.trash/ (the converter ignores it), so a delete can be
undone with `restore` until `empty-trash` is run.

Usage (FOLDER is a task folder such as vla_data/pick_up_the_cup, or vla_data for a summary):
    python3 vla/manage_episodes.py FOLDER                    list episodes (default command)
    python3 vla/manage_episodes.py FOLDER delete last        move the newest episode to the trash
    python3 vla/manage_episodes.py FOLDER delete 3 5 8-10    by episode number (ranges allowed)
    python3 vla/manage_episodes.py FOLDER delete failed      every episode not marked success
    python3 vla/manage_episodes.py FOLDER restore            bring back the most recently deleted
    python3 vla/manage_episodes.py FOLDER restore all
    python3 vla/manage_episodes.py FOLDER trash              list the trash
    python3 vla/manage_episodes.py FOLDER empty-trash        delete the trash for good
    python3 vla/manage_episodes.py FOLDER mark 4 failure     change status (success / failure)
    python3 vla/manage_episodes.py FOLDER task 4 "new instruction text"

delete and empty-trash ask for confirmation; -y skips it.
"""

import argparse
import json
import os
import re
import shutil
import sys
import time

TRASH = ".trash"
EPISODE_RE = re.compile(r"episode_(\d+)$")
TRASH_RE = re.compile(r"(episode_\d+)__(\d{8}_\d{6}(?:_\d+)?)$")


# ----------------------------------------------------------------------------- helpers

def read_meta(ep_dir):
    try:
        with open(os.path.join(ep_dir, "meta.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def write_meta(ep_dir, meta):
    tmp = os.path.join(ep_dir, "meta.json.tmp")
    with open(tmp, "w") as f:
        json.dump(meta, f, indent=2)
    os.replace(tmp, os.path.join(ep_dir, "meta.json"))


def list_episodes(folder):
    """[(number, path, meta)] sorted by number."""
    if not os.path.isdir(folder):
        return []
    out = []
    for name in os.listdir(folder):
        m = EPISODE_RE.match(name)
        if m and os.path.isdir(os.path.join(folder, name)):
            out.append((int(m.group(1)), os.path.join(folder, name), read_meta(os.path.join(folder, name))))
    return sorted(out)


def list_trash(folder):
    """[(deleted_stamp, trash_path, original_name, meta)] oldest first."""
    tdir = os.path.join(folder, TRASH)
    if not os.path.isdir(tdir):
        return []
    out = []
    for name in os.listdir(tdir):
        m = TRASH_RE.match(name)
        if m:
            out.append((m.group(2), os.path.join(tdir, name), m.group(1), read_meta(os.path.join(tdir, name))))
    return sorted(out)


def trash_episode(ep_dir):
    """Move an episode folder into the trash; returns the trash path."""
    folder, name = os.path.split(os.path.normpath(ep_dir))
    tdir = os.path.join(folder, TRASH)
    os.makedirs(tdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    dest, k = os.path.join(tdir, "{}__{}".format(name, stamp)), 1
    while os.path.exists(dest):
        dest, k = os.path.join(tdir, "{}__{}_{}".format(name, stamp, k)), k + 1
    os.rename(ep_dir, dest)
    return dest


def restore_episode(folder, trash_path, original):
    """Move a trashed episode back; uses the next free number if its own is taken."""
    dest = os.path.join(folder, original)
    if os.path.exists(dest):
        eps = list_episodes(folder)
        dest = os.path.join(folder, "episode_{:06d}".format(eps[-1][0] + 1 if eps else 0))
    os.rename(trash_path, dest)
    return dest


def describe(number, meta):
    return "{:6d}  {:10s} {:5s} {:6s}  {:19s}  {}".format(
        number, meta.get("status", "?"), str(meta.get("frames", "?")),
        "{:.1f}s".format(meta["duration_s"]) if "duration_s" in meta else "?",
        meta.get("started", ""), meta.get("task", ""))


def summary_line(episodes):
    counts = {}
    for _, _, m in episodes:
        counts[m.get("status", "?")] = counts.get(m.get("status", "?"), 0) + 1
    frames = sum(m.get("frames", 0) or 0 for _, _, m in episodes)
    secs = sum(m.get("duration_s", 0) or 0 for _, _, m in episodes)
    return "{} episodes ({}), {} frames, {:.1f} min".format(
        len(episodes), ", ".join("{} {}".format(v, k) for k, v in sorted(counts.items())) or "none",
        frames, secs / 60)


def parse_numbers(tokens, episodes):
    have = {n for n, _, _ in episodes}
    if tokens == ["last"]:
        return [episodes[-1][0]] if episodes else []
    if tokens == ["failed"]:
        return [n for n, _, m in episodes if m.get("status") != "success"]
    picked = []
    for tok in tokens:
        m = re.match(r"^(\d+)(?:-(\d+))?$", tok)
        if not m:
            sys.exit("not an episode number or range: {}".format(tok))
        lo, hi = int(m.group(1)), int(m.group(2) or m.group(1))
        picked += [n for n in range(lo, hi + 1) if n in have]
        missing = [n for n in range(lo, hi + 1) if n not in have]
        if missing and lo == hi:
            sys.exit("no episode {}".format(lo))
    return sorted(set(picked))


def confirm(question, assume_yes):
    if assume_yes:
        return True
    try:
        return input(question + " [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


# ----------------------------------------------------------------------------- commands

def cmd_list(folder):
    episodes = list_episodes(folder)
    if not episodes:
        # a vla_data root: one line per task folder
        tasks = [d for d in sorted(os.listdir(folder)) if list_episodes(os.path.join(folder, d))] \
            if os.path.isdir(folder) else []
        if not tasks:
            print("No episodes in {}".format(folder))
            return
        for d in tasks:
            print("{:40s} {}".format(d, summary_line(list_episodes(os.path.join(folder, d)))))
        return
    print("{:>6s}  {:10s} {:5s} {:6s}  {:19s}  {}".format("ep", "status", "frms", "length", "recorded", "task"))
    for n, _, m in episodes:
        print(describe(n, m))
    print(summary_line(episodes))
    trash = list_trash(folder)
    if trash:
        print("{} in the trash (restore / empty-trash)".format(len(trash)))


def cmd_delete(folder, tokens, assume_yes):
    episodes = list_episodes(folder)
    if not tokens:
        sys.exit("delete what? last | failed | numbers, e.g. delete 3 5 8-10")
    numbers = parse_numbers(tokens, episodes)
    if not numbers:
        print("Nothing to delete.")
        return
    by_number = {n: (p, m) for n, p, m in episodes}
    for n in numbers:
        print(describe(n, by_number[n][1]))
    if not confirm("Move {} episode(s) to the trash?".format(len(numbers)), assume_yes):
        print("Cancelled.")
        return
    for n in numbers:
        trash_episode(by_number[n][0])
    print("Moved {} episode(s) to {} (undo: restore).".format(len(numbers), os.path.join(folder, TRASH)))


def cmd_restore(folder, tokens):
    trash = list_trash(folder)
    if not trash:
        print("The trash is empty.")
        return
    if tokens == ["all"]:
        picked = trash
    elif tokens:
        picked = [t for t in trash if t[2] in ["episode_{:06d}".format(int(x)) for x in tokens if x.isdigit()]]
        if not picked:
            sys.exit("not in the trash: {}".format(" ".join(tokens)))
    else:
        picked = [trash[-1]]                              # the most recently deleted
    for _, path, original, _ in picked:
        dest = restore_episode(folder, path, original)
        print("restored {} -> {}".format(original, os.path.basename(dest)))


def cmd_trash(folder):
    trash = list_trash(folder)
    if not trash:
        print("The trash is empty.")
        return
    for stamp, _, original, meta in trash:
        print("deleted {}  {}".format(stamp, describe(int(original.split("_")[1]), meta)))


def cmd_empty_trash(folder, assume_yes):
    trash = list_trash(folder)
    if not trash:
        print("The trash is empty.")
        return
    if confirm("Permanently delete {} episode(s) in the trash?".format(len(trash)), assume_yes):
        shutil.rmtree(os.path.join(folder, TRASH))
        print("Trash emptied.")


def cmd_set(folder, tokens, key):
    if len(tokens) < 2:
        sys.exit("usage: {} NUMBER VALUE".format("mark" if key == "status" else "task"))
    episodes = {n: p for n, p, _ in list_episodes(folder)}
    if not tokens[0].isdigit() or int(tokens[0]) not in episodes:
        sys.exit("no episode {}".format(tokens[0]))
    value = " ".join(tokens[1:])
    if key == "status" and value not in ("success", "failure"):
        sys.exit("status must be success or failure")
    path = episodes[int(tokens[0])]
    meta = read_meta(path)
    meta[key] = value
    write_meta(path, meta)
    print(describe(int(tokens[0]), meta))


def main():
    parser = argparse.ArgumentParser(description="Manage recorded VLA episodes.",
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("folder", help="task folder (vla_data/<task>) or vla_data")
    parser.add_argument("command", nargs="?", default="list",
                        choices=["list", "delete", "restore", "trash", "empty-trash", "mark", "task"])
    parser.add_argument("args", nargs="*")
    parser.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")
    a = parser.parse_args()
    if a.command == "list":
        cmd_list(a.folder)
    elif a.command == "delete":
        cmd_delete(a.folder, a.args, a.yes)
    elif a.command == "restore":
        cmd_restore(a.folder, a.args)
    elif a.command == "trash":
        cmd_trash(a.folder)
    elif a.command == "empty-trash":
        cmd_empty_trash(a.folder, a.yes)
    elif a.command == "mark":
        cmd_set(a.folder, a.args, "status")
    else:
        cmd_set(a.folder, a.args, "task")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Write the BLE copy of a touchpad's report descriptor.

A GATT attribute value is at most 512 bytes (Core spec Vol 3, Part F,
3.2.9), and a HOGP host may stop reading the Report Map there. This reads
report-descriptor from a zmk,hid-touchpad devicetree file and writes
ble-report-descriptor and ble-feature-report-ids next to it:

- top-level collections on a --drop-page usage page are left out;
- global items that set a value already in effect are left out (globals
  carry over from item to item in HID), and every value gets its shortest
  encoding that reads the same signed and unsigned.

Every input, output and feature item that remains parses exactly as in
report-descriptor: same global and local state, same enclosing
collections. The script checks that, and that the result fits in 512 bytes.

The descriptor must have one item per line, `05 01  // comment`, as in the
Daisy dtsi. Comments are kept.

usage: ble_report_map.py FILE [--drop-page 0xff00 ...] [--check]
"""

import argparse
import difflib
import re
import sys

MAX_ATTRIBUTE_LEN = 512
MAIN, GLOBAL, LOCAL = 0, 1, 2
INPUT, OUTPUT, COLLECTION, FEATURE, END_COLLECTION = 8, 9, 10, 11, 12
USAGE_PAGE, REPORT_ID, PUSH, POP = 0, 8, 10, 11
SIGNED = {1, 2, 3, 4}  # logical/physical minimum/maximum

USB_PROP = re.compile(r"(?<![\w-])report-descriptor\s*=\s*\[(.*?)\];", re.S)
BLE_PROP = re.compile(r"ble-report-descriptor\s*=\s*\[.*?\];", re.S)
BLE_FEATURES = re.compile(r"ble-feature-report-ids\s*=\s*<[^>]*>;")


def fail(msg):
    sys.exit(f"ble_report_map: {msg}")


class Item:
    def __init__(self, raw, line):
        self.raw = raw
        self.line = line  # source line, for its indent and comment
        prefix = raw[0]
        self.type, self.tag = (prefix >> 2) & 3, prefix >> 4
        n = len(raw) - 1
        data = int.from_bytes(raw[1:], "little")
        if self.type == GLOBAL and self.tag in SIGNED and n and data >> (8 * n - 1):
            data -= 1 << (8 * n)
        self.value = data

    def is_data(self):
        return self.type == MAIN and self.tag in (INPUT, OUTPUT, FEATURE)


def parse(body):
    items = []
    for line in body.split("\n"):
        code = line.split("//", 1)[0]
        if "/*" in code:
            fail(f"block comment in the descriptor: {line.strip()}")
        raw = bytes(int(b, 16) for b in code.split())
        if not raw:
            continue
        if raw[0] == 0xFE or len(raw) != 1 + (0, 1, 2, 4)[raw[0] & 3]:
            fail(f"expected one short item per line: {line.strip()}")
        items.append(Item(raw, line))
    return items


def encode(item, value):
    """Shortest encoding of a global item's value. Min/max values stay in
    the signed range of their size, so parsers that read them as unsigned
    agree."""
    for n, code in ((1, 1), (2, 2), (4, 3)):
        top = 1 << (8 * n - 1) if item.tag in SIGNED else 1 << (8 * n)
        low = -top if item.tag in SIGNED else 0
        if low <= value < top:
            return bytes([(item.tag << 4) | (GLOBAL << 2) | code]) + (
                value & ((1 << (8 * n)) - 1)).to_bytes(n, "little")
    fail(f"value {value} doesn't fit an item")


def reports(items):
    """{(kind, report ID): [what each data item of that report sees]}."""
    glob, stack, local, path, out = {}, [], [], [], {}
    for it in items:
        if it.type == GLOBAL:
            if it.tag == PUSH:
                stack.append(dict(glob))
            elif it.tag == POP:
                glob = stack.pop()
            else:
                glob[it.tag] = it.value
        elif it.type == LOCAL:
            local.append((it.tag, it.value))
        else:
            if it.tag == COLLECTION:
                path.append((it.value, glob.get(USAGE_PAGE), tuple(local)))
            elif it.tag == END_COLLECTION:
                path.pop()
            else:
                seen = {k: v for k, v in glob.items() if k != REPORT_ID}
                out.setdefault((it.tag, glob.get(REPORT_ID, 0)), []).append(
                    (it.value, tuple(sorted(seen.items())), tuple(local), tuple(path)))
            local = []
    return out


def ble_items(items, drop_pages):
    # Top-level collections to leave out, with the items that lead up to them
    keep, pending, depth, page = [], [], 0, None
    for it in items:
        if it.type == GLOBAL and it.tag == USAGE_PAGE:
            page = it.value
        pending.append(it)
        if it.type == MAIN and it.tag == COLLECTION:
            if depth == 0:
                top_page = page
            depth += 1
        elif it.type == MAIN and it.tag == END_COLLECTION:
            depth -= 1
            if depth == 0:
                if top_page not in drop_pages:
                    keep += pending
                pending = []
    keep += pending

    # Drop globals that change nothing, and shorten the rest
    out, glob, stack = [], {}, []
    for it in keep:
        if it.type == GLOBAL and it.tag == PUSH:
            stack.append(dict(glob))
        elif it.type == GLOBAL and it.tag == POP:
            glob = stack.pop()
        elif it.type == GLOBAL:
            if it.tag != REPORT_ID and glob.get(it.tag) == it.value:
                continue
            glob[it.tag] = it.value
            it = Item(encode(it, it.value), it.line)
        out.append(it)
    return out


def render(items):
    lines = []
    for it in items:
        code, sep, comment = it.line.partition("//")
        indent = code[:len(code) - len(code.lstrip())]
        width = len(code) - len(indent)
        lines.append(f"{indent}{it.raw.hex(' ').upper():<{width}}{sep}{comment}".rstrip())
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--drop-page", type=lambda s: int(s, 0), action="append", default=[],
                    help="usage page of a top-level collection to leave out (repeatable)")
    ap.add_argument("--check", action="store_true", help="fail if FILE is out of date")
    args = ap.parse_args()

    text = open(args.file).read()
    m = USB_PROP.search(text)
    if not m:
        fail(f"no report-descriptor in {args.file}")
    usb = parse(m.group(1))
    ble = ble_items(usb, set(args.drop_page))

    usb_reports, ble_reports = reports(usb), reports(ble)
    for key, seen in ble_reports.items():
        if usb_reports.get(key) != seen:
            fail(f"report {key[1]} doesn't parse the same as in report-descriptor")
    size = sum(len(it.raw) for it in ble)
    if size > MAX_ATTRIBUTE_LEN:
        fail(f"the BLE copy is {size} bytes, over {MAX_ATTRIBUTE_LEN}")

    m_ids = re.search(r"(?<![\w-])feature-report-ids\s*=\s*<([^>]*)>;", text)
    if not m_ids:
        fail(f"no feature-report-ids in {args.file}")
    ble_features = {rid for kind, rid in ble_reports if kind == FEATURE}
    ids = [i for i in m_ids.group(1).split() if int(i, 0) in ble_features]

    features = f"ble-feature-report-ids = <{' '.join(ids)}>;"
    descriptor = f"ble-report-descriptor = [\n{render(ble)}\n];"
    pages = " ".join(f"--drop-page {p:#06x}" for p in args.drop_page)
    new = text
    if BLE_PROP.search(new):
        new = BLE_PROP.sub(lambda _: descriptor, new)
        new = BLE_FEATURES.sub(lambda _: features, new)
    else:
        dropped = sorted({rid for _, rid in usb_reports} - {rid for _, rid in ble_reports})
        note = f", without reports {', '.join(map(str, dropped))}" if dropped else ""
        new = new[:m.end()] + f"""

/* BLE copy of report-descriptor{note}, re-encoded to fit
 * the {MAX_ATTRIBUTE_LEN}-byte GATT attribute limit. Every report that remains parses as
 * above; global items that repeat a value already in effect are left out.
 * Written by zephyr-hid-touchpad-module/scripts/ble_report_map.py{' ' + pages if pages else ''};
 * edit report-descriptor and rerun it instead of editing this by hand. */
{features}
{descriptor}""" + new[m.end():]

    if args.check:
        if new != text:
            sys.stdout.writelines(difflib.unified_diff(
                text.splitlines(True), new.splitlines(True), args.file, "regenerated"))
            fail(f"{args.file} is out of date; rerun without --check")
        print(f"{args.file}: BLE report map up to date ({size} bytes)")
        return
    open(args.file, "w").write(new)
    print(f"{args.file}: BLE report map {size} bytes (report-descriptor "
          f"{sum(len(it.raw) for it in usb)}), features {' '.join(ids)}")


if __name__ == "__main__":
    main()

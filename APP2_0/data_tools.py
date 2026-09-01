"""Export relace do CSV a načítání uložených textových logů (pro historický prohlížeč)."""

import csv


def export_session_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=";")
        writer.writerow(header)
        writer.writerows(rows)


def parse_log_file(path):
    """Načte tab-oddělený log (hlavička + řádky) a vrátí (header, {sloupec: [hodnoty]})."""
    with open(path, "r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f if line.strip()]
    if not lines:
        return [], {}

    header = lines[0].split("\t")
    columns = {name: [] for name in header}
    for line in lines[1:]:
        parts = line.split("\t")
        if len(parts) != len(header):
            continue
        for name, value in zip(header, parts):
            columns[name].append(value)
    return header, columns

#!/usr/bin/env python3

import argparse
import csv
import pathlib


def to_float(value: str) -> str:
    value = value.strip()
    if value == "" or value.lower() == "nan":
        return "-1.0"
    try:
        # Preserve as-is numeric string representation
        float(value)
        return value
    except Exception:
        return "-1.0"


def to_int(value: str) -> str:
    value = value.strip()
    if value == "" or value.lower() == "nan":
        return "-1"
    try:
        return str(int(round(float(value))))
    except Exception:
        return "-1"


def to_cxx_string_literal(value: str) -> str:
    # Return a well-escaped C++ string literal
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def generate_header(input_csv: pathlib.Path, out_header: pathlib.Path) -> None:
    with input_csv.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    n = len(rows)

    lines = []
    lines.append("#pragma once\n")
    lines.append('#include "Simulation/Data/Element.hpp"\n')
    lines.append("#include <cstddef>\n\n")
    lines.append("namespace Honeycomb::Data {\n\n")
    lines.append(f"inline constexpr std::size_t kElementCount = {n};\n")
    lines.append("inline constexpr Element kPeriodicTable[kElementCount] = {\n")

    for row in rows:
        atomic_number = to_int(row.get("AtomicNumber", ""))
        symbol = to_cxx_string_literal(row.get("Symbol", ""))
        name = to_cxx_string_literal(row.get("Name", ""))
        atomic_mass = to_float(row.get("AtomicMass(u)", ""))
        cpk_hex = to_cxx_string_literal(row.get("CPKHexColor", ""))
        econf = to_cxx_string_literal(row.get("ElectronConfiguration", ""))
        en = to_float(row.get("Electronegativity(Pauling)", ""))
        vdw = to_int(row.get("AtomicRadius(vdWaals)", ""))
        ion = to_float(row.get("IonizationEnergy(eV)", ""))
        ea = to_float(row.get("ElectronAffinity(eV)", ""))
        ox = to_cxx_string_literal(row.get("OxidationStates", ""))
        state = to_cxx_string_literal(row.get("StandardState", ""))
        melt = to_float(row.get("MeltingPoint(K)", ""))
        boil = to_float(row.get("BoilingPoint(K)", ""))
        density = to_float(row.get("Density(g/cm3)", ""))
        group = to_cxx_string_literal(row.get("GroupBlock", ""))
        year = to_cxx_string_literal(row.get("YearDiscovered", ""))

        lines.append(
            "    { "
            + ", ".join(
                [
                    atomic_number,
                    symbol,
                    name,
                    atomic_mass,
                    cpk_hex,
                    econf,
                    en,
                    vdw,
                    ion,
                    ea,
                    ox,
                    state,
                    melt,
                    boil,
                    density,
                    group,
                    year,
                ]
            )
            + " },\n"
        )

    lines.append("};\n\n")
    lines.append("}  // namespace Honeycomb::Data\n")

    out_header.parent.mkdir(parents=True, exist_ok=True)
    out_header.write_text("".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(
        description="Generate a C++ header from periodic_table.csv"
    )
    parser.add_argument("--input", required=True, help="Path to periodic_table.csv")
    parser.add_argument("--out-header", required=True, help="Output header path")
    args = parser.parse_args()

    generate_header(pathlib.Path(args.input), pathlib.Path(args.out_header))


if __name__ == "__main__":
    main()

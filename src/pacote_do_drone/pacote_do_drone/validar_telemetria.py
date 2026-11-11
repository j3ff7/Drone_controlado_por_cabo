import argparse
import csv
import hashlib
import math
import re
from pathlib import Path

from scipy.optimize import root


GRAVITY = 9.81


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def trig_position(length, alpha_d, beta_d):
    horizontal = length * math.cos(beta_d)
    return (
        horizontal * math.cos(alpha_d),
        horizontal * math.sin(alpha_d),
        length * math.sin(beta_d),
    )


def point(row, prefix):
    return tuple(float(row[f"{prefix}_{axis}"]) for axis in "xyz")


def horizontal_force(row, prefix):
    force_x = float(row[f"{prefix}_force_x"])
    force_y = float(row[f"{prefix}_force_y"])
    return math.hypot(force_x, force_y)


def input_digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def numbered_output_for_input(input_path, directory, stem, suffix):
    digest = input_digest(input_path)
    pattern = re.compile(rf"^{re.escape(stem)}_(\d+){re.escape(suffix)}$")
    candidates = []

    for candidate in directory.glob(f"{stem}_*{suffix}"):
        match = pattern.match(candidate.name)
        if match:
            candidates.append((int(match.group(1)), candidate))

    for number, candidate in sorted(candidates):
        try:
            with candidate.open(newline="", encoding="utf-8") as source:
                row = next(csv.DictReader(source))
        except (OSError, StopIteration, csv.Error):
            continue

        if row.get("input_sha256") == digest:
            return candidate, number, digest

        if (
            row.get("input_file") == str(input_path)
            and "input_sha256" not in row
            and candidate.stat().st_mtime >= input_path.stat().st_mtime
        ):
            return candidate, number, digest

    number = 1
    while True:
        candidate = directory / f"{stem}_{number:03d}{suffix}"
        if not candidate.exists():
            return candidate, number, digest
        number += 1


def latest_telemetry_path(directory):
    paths = sorted(directory.glob("telemetria_*.csv"))
    numbered = [
        path for path in paths
        if re.fullmatch(r"telemetria_\d{3}\.csv", path.name)
    ]
    if numbered:
        return numbered[-1]

    fallback = directory / "telemetria.csv"
    if fallback.exists():
        return fallback
    return fallback


def catenary_from_geometry(length, horizontal_span, vertical_span, beta_d):
    if horizontal_span <= 0.0 or length <= horizontal_span:
        raise ValueError("A geometria não define uma catenária frouxa.")

    initial = (0.0, math.asinh(math.tan(beta_d)))

    def equations(values):
        u0, u1 = values
        delta = u1 - u0
        if abs(delta) < 1e-12:
            return (length, vertical_span)
        scale = horizontal_span / delta
        return (
            scale * (math.sinh(u1) - math.sinh(u0)) - length,
            scale * (math.cosh(u1) - math.cosh(u0)) - vertical_span,
        )

    solution = root(equations, initial)
    if not solution.success:
        raise ValueError(
            f"A solução da catenária não convergiu: {solution.message}"
        )

    u0, u1 = solution.x
    scale = horizontal_span / (u1 - u0)
    return (
        math.atan(math.sinh(u0)),
        math.atan(math.sinh(u1)),
        scale,
    )


def validate(
    input_path,
    output_path,
    linear_density,
    test_number,
    input_sha256,
):
    with input_path.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))

    if not rows:
        raise ValueError(f"O CSV não contém amostras: {input_path}")

    required = {
        "tether_length_m",
        "alpha_d_rad",
        "beta_d_rad",
        "alpha_p_rad",
        "beta_p_rad",
        "cable_root_x",
        "cable_root_y",
        "cable_root_z",
        "rover_connection_x",
        "rover_connection_y",
        "rover_connection_z",
        "anchor_force_x",
        "anchor_force_y",
        "tip_force_x",
        "tip_force_y",
    }
    missing = required - set(rows[0])
    if missing:
        names = ", ".join(sorted(missing))
        raise ValueError(f"Colunas ausentes no CSV: {names}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "test_number",
        "input_file",
        "input_sha256",
        "time_s",
        "measured_x",
        "measured_y",
        "measured_z",
        "trig_x",
        "trig_y",
        "trig_z",
        "trig_error_m",
        "catenary_lambda_m",
        "catenary_horizontal_tension_n",
        "beta_d_theoretical_rad",
        "beta_p_theoretical_rad",
        "t0_measured_n",
        "beta_d_error_rad",
        "beta_p_error_rad",
        "t0_error_n",
    ]

    errors = {
        "trig_error_m": [],
        "beta_d_error_rad": [],
        "beta_p_error_rad": [],
        "t0_error_n": [],
    }
    with output_path.open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader()

        for row in rows:
            length = float(row["tether_length_m"])
            # The collector stores the drone tangent in the opposite direction
            # from the article's platform-to-drone localization convention.
            alpha_d = wrap_angle(float(row["alpha_d_rad"]) + math.pi)
            beta_d = -float(row["beta_d_rad"])

            measured = tuple(
                drone - platform
                for drone, platform in zip(
                    point(row, "cable_root"),
                    point(row, "rover_connection"),
                )
            )
            predicted_trig = trig_position(length, alpha_d, beta_d)
            trig_error = math.dist(measured, predicted_trig)
            errors["trig_error_m"].append(trig_error)

            horizontal_span = math.hypot(measured[0], measured[1])
            beta_p_theoretical, beta_d_theoretical, scale = (
                catenary_from_geometry(
                    length,
                    horizontal_span,
                    measured[2],
                    beta_d,
                )
            )
            t0_theoretical = scale * linear_density * GRAVITY
            t0_measured = (
                horizontal_force(row, "anchor")
                + horizontal_force(row, "tip")
            ) / 2.0
            beta_d_error = beta_d - beta_d_theoretical
            beta_p_error = (
                float(row["beta_p_rad"]) - beta_p_theoretical
            )
            t0_error = t0_measured - t0_theoretical
            errors["beta_d_error_rad"].append(abs(beta_d_error))
            errors["beta_p_error_rad"].append(abs(beta_p_error))
            errors["t0_error_n"].append(abs(t0_error))

            writer.writerow(
                {
                    "test_number": f"{test_number:03d}",
                    "input_file": str(input_path),
                    "input_sha256": input_sha256,
                    "time_s": row["time_s"],
                    "measured_x": measured[0],
                    "measured_y": measured[1],
                    "measured_z": measured[2],
                    "trig_x": predicted_trig[0],
                    "trig_y": predicted_trig[1],
                    "trig_z": predicted_trig[2],
                    "trig_error_m": trig_error,
                    "catenary_lambda_m": scale,
                    "catenary_horizontal_tension_n": t0_theoretical,
                    "beta_d_theoretical_rad": beta_d_theoretical,
                    "beta_p_theoretical_rad": beta_p_theoretical,
                    "t0_measured_n": t0_measured,
                    "beta_d_error_rad": beta_d_error,
                    "beta_p_error_rad": beta_p_error,
                    "t0_error_n": t0_error,
                }
            )

    print(f"Amostras validadas: {len(rows)}")
    units = {
        "trig_error_m": "m",
        "beta_d_error_rad": "rad",
        "beta_p_error_rad": "rad",
        "t0_error_n": "N",
    }
    for name, values in errors.items():
        print(
            f"{name}: média={sum(values) / len(values):.6f} "
            f"{units[name]}, máximo={max(values):.6f} {units[name]}"
        )
    print(f"Resultado: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Valida a telemetria usando as equações do artigo."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--linear-density",
        type=float,
        default=0.04,
        help="Densidade linear do cabo em kg/m (padrão: 0.04).",
    )
    args = parser.parse_args()
    if args.input is None:
        args.input = latest_telemetry_path(Path("results/validation"))

    if args.output is None:
        output, test_number, input_sha256 = numbered_output_for_input(
            args.input,
            args.input.parent,
            "validacao_teorica",
            ".csv",
        )
    else:
        output = args.output
        test_number = 0
        input_sha256 = input_digest(args.input)

    validate(
        args.input,
        output,
        args.linear_density,
        test_number,
        input_sha256,
    )


if __name__ == "__main__":
    main()

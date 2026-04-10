import csv
import sys

CSV_PATH = "src/main/deploy/calibration/turret_calibration_data.csv"
RPM_BUMP = 50

rows = []
with open(CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)
    fieldnames = reader.fieldnames
    for row in reader:
        row["flywheel_rpm"] = str(int(row["flywheel_rpm"]) + RPM_BUMP)
        rows.append(row)

with open(CSV_PATH, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

print(f"Bumped flywheel_rpm by {RPM_BUMP} for {len(rows)} rows.")

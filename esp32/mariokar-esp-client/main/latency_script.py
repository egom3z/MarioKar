import re
import csv

# Input log file
INPUT_FILE = "latency_logs.txt"
OUTPUT_CSV = "rtt_results.csv"

# Regex patterns
distance_pattern = re.compile(r"###\s*([\d\.]+)\s*feet", re.IGNORECASE)
rtt_pattern = re.compile(r"RTT\s*=\s*([\d\.]+)")

results = {}
current_distance = None

with open(INPUT_FILE, "r") as f:
    for line in f:
        # Detect distance marker
        dist_match = distance_pattern.search(line)
        if dist_match:
            current_distance = float(dist_match.group(1))
            results[current_distance] = []
            continue
        
        # Extract RTT values
        if current_distance is not None:
            rtt_match = rtt_pattern.findall(line)
            for val in rtt_match:
                results[current_distance].append(float(val))

# Determine max number of RTT columns
max_len = max(len(vals) for vals in results.values())

# Write CSV
with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.writer(f)
    
    # Header: Distance, RTT1, RTT2, RTT3, ...
    header = ["Distance (ft)"] + [f"RTT{i+1}" for i in range(max_len)]
    writer.writerow(header)
    
    # Rows
    for dist, rtts in sorted(results.items()):
        row = [dist] + rtts + [""] * (max_len - len(rtts))
        writer.writerow(row)

print(f"Done! CSV saved to {OUTPUT_CSV}")

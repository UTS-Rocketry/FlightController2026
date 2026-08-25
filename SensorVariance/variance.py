import statistics
with open('baro_noise.txt') as f:
    vals = [float(x) for x in f if x.strip() and not x.startswith('Flight') ]  # skip any stray text lines
mean = statistics.mean(vals)
std  = statistics.stdev(vals)
print(f"n={len(vals)} mean={mean:.3f}m std={std:.3f}m variance={std**2:.3f}")
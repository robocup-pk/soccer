import re

# Test parsing P-matrix from actual log
lines = """P-Matrix after update:
9.999e-05         0         0
        0 9.999e-05         0
        0         0 0.0984499""".split('\n')

for i, line in enumerate(lines):
    print(f"Line {i}: '{line}'")
    if 'P-Matrix after update:' in line:
        p_matrix = []
        for k in range(1, 4):
            if i + k < len(lines):
                print(f"  Parsing line {i+k}: '{lines[i+k]}'")
                p_row = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', lines[i+k])
                print(f"  Found values: {p_row}")
                if len(p_row) >= 3:
                    p_matrix.append([float(x) for x in p_row[:3]])
        print(f"Final P-matrix: {p_matrix}")

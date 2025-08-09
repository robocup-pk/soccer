import re

def parse_kalman_log(filename='kalman_detailed_log.txt'):
    """Parse the detailed Kalman filter log"""
    
    data = {
        'predictions': [],
        'updates': [],
    }
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    print(f"Total lines in file: {len(lines)}")
    
    i = 0
    update_count = 0
    while i < len(lines):
        line = lines[i]
        
        # Parse update steps
        if 'UPDATE STEP (Camera Measurement)' in line:
            update_count += 1
            match = re.search(r'#(\d+)', line)
            if match:
                step_num = int(match.group(1))
                update_data = {'step': step_num}
                
                # Look for P-Matrix after update within next 30 lines
                found_p = False
                for j in range(1, min(30, len(lines) - i)):
                    next_line = lines[i + j]
                    
                    if 'P-Matrix after update:' in next_line:
                        # Read next 3 lines for P-matrix
                        p_matrix = []
                        for k in range(1, 4):
                            if i + j + k < len(lines):
                                p_row = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', lines[i + j + k])
                                if len(p_row) >= 3:
                                    p_matrix.append([float(x) for x in p_row[:3]])
                        if len(p_matrix) == 3:
                            update_data['P_after'] = p_matrix
                            found_p = True
                            print(f"Update #{step_num}: Found P-matrix with diagonal [{p_matrix[0][0]:.2e}, {p_matrix[1][1]:.2e}, {p_matrix[2][2]:.2e}]")
                        break
                
                if not found_p:
                    print(f"Update #{step_num}: No P-matrix found")
                
                if 'P_after' in update_data:
                    data['updates'].append(update_data)
        
        i += 1
    
    print(f"Total UPDATE steps found: {update_count}")
    print(f"Updates with P-matrix: {len(data['updates'])}")
    
    return data

# Test the parser
data = parse_kalman_log()

import re
import os

# File to process
input_file = "nvs_stuff.h"     # Change this to your actual file
output_file = "new_nvs_stuff.h"

# Define replacements (regex-safe)
replacements = {
    r'\bKp\b': 'nvsSettingsStorage.Kp',
    r'\bKi\b': 'nvsSettingsStorage.Ki',
    r'\bKd\b': 'nvsSettingsStorage.Kd',
    r'\bLPF_ALPHA\b': 'nvsSettingsStorage.LPF_ALPHA',
    r'\bscaleA\b': 'nvsSettingsStorage.scaleA',
    r'\bscaleB\b': 'nvsSettingsStorage.scaleB',
    r'\bQ\b': 'nvsSettingsStorage.Q'
}

# Read file
with open(input_file, 'r') as f:
    content = f.read()

# Apply replacements
for pattern, replacement in replacements.items():
    content = re.sub(pattern, replacement, content)

# Save new file
with open(output_file, 'w') as f:
    f.write(content)

print(f"✅ Finished. Output written to {output_file}")

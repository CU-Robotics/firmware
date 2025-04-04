import re
import os
import sys

"""
python script to parse the map file (generated by build process) and find what symbols are using the most ITCM/DTCM

from firmware/:
    python tools/tcm_blame.py ./build/firmware.map 20
    python3 tools/tcm_blame.py ./build/firmware.map 20

from firmware/tools:
    python tcm_blame.py ../build/firmware.map 20 
    python3 tcm_blame.py ../build/firmware.map 20 
"""

# origin and end addresses for tcm (Teensy 4.1)
ITCM_START, ITCM_END = 0x00000000, 0x0007FFFF
DTCM_START, DTCM_END = 0x20000000, 0x2007FFFF

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <map_file_path> <top_n>")
    sys.exit(1)

map_file_path = sys.argv[1]
n = int(sys.argv[2])

symbols = []
line_num = 0
is_discarded = False
is_debug = False

# parse map file
with open(map_file_path, "r") as map_file:
    for line in map_file:
        # skip the discarded input section
        if (line == "Discarded input sections\n"):
            is_discarded = True
        # end of the discarded input section
        if (line == "Memory Configuration\n"):
            is_discarded = False

        line_num += 1
        if (is_discarded):
            continue

        # skip the debug sections
        debug_match = re.match(r'^\s*\.debug_', line)
        if (debug_match):
            is_debug = True
        # end of the debug sections
        if (line == "Cross Reference Table\n"):
            is_debug = False
        
        if (is_debug):
            continue
        
        # regex from hell
        match = re.match(r"\s*(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(\S+)", line)

        # if we get a match for this line, parse it
        if match:
            address = int(match.group(1), 16)  # <- int base 16 lol
            size = int(match.group(2), 16)
            symbol = match.group(3)

            # infer the region by address
            if ITCM_START <= address <= ITCM_END:
                region = "ITCM"
            elif DTCM_START <= address <= DTCM_END:
                region = "DTCM"
            else:
                # skip things that arent TCM
                continue

            symbols.append((region, address, size, symbol, line_num))

# sort by size and get the top n
dtcm_symbols = sorted([s for s in symbols if s[0] == "DTCM"], key=lambda x: -x[2])[:n]
itcm_symbols = sorted([s for s in symbols if s[0] == "ITCM"], key=lambda x: -x[2])[:n]

# thank you chat
def print_table(title, symbol_list):
    print(f"\nTop {n} {title} Memory Users:\n")
    print(f"{'Region':<6} {'Addr':>10} {'Size':>8}  {'Symbol':<30} {'Line'}")
    print("-" * 80)
    for region, address, size, symbol, line_num in symbol_list:
        filename = os.path.basename(symbol)
        print(
            f"{region:<6} {address:#010x} {size:8}  {filename:<30} {map_file_path}:{line_num}"
        )


print_table("DTCM", dtcm_symbols)
print_table("ITCM", itcm_symbols)

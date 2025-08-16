# Data on test runs
# Keys "A" and "B" may be used in file names for export
runData = {
    "baseline1":  {"A": "F9T-Bob", "notes": "Config left over from previous experiments, run 1"},
    "baseline2":  {"A": "F9T-Bob", "notes": "Config left over from previous experiments, run 2"},
    "baseline3":  {"A": "F9T-Bob", "notes": "Config left over from previous experiments, with power cycle"},
    "baseline4":  {"A": "F9T-Bob", "notes": "Factory reset, but didn't clear flash for BBR keys"},
    "baseline5":  {"A": "F9T-Bob", "notes": "Factory reset, and cleared flash for BBR keys"},
    "fixedPos1":  {"A": "F9T-Bob", "notes": "First working fixed position, no reset"},
    "fixedPos2":  {"A": "F9T-Bob", "notes": "Factory reset, then fixed position"},
    "fixedL1ca1": {"A": "F9T-Bob", "notes": "Fixed position, L1 C/A signal only, run 1"},
    "fixedL1ca2": {"A": "F9T-Bob", "notes": "Fixed position, L1 C/A signal only, run 2"},
    "fixedL1l51": {"A": "F9T-Bob", "notes": "Fixed position, L1 C/A and L5 signals, run 1"},
    # I think the incorrect host timestamps on fixedL1l52 my correlate incorrectly with epoch seconds. Danger!
    "fixedL1l52": {"A": "F9T-Bob", "B": "F9T-PT", "notes": "Fixed position, L1 C/A and L5 signals, run 2, short, wrong host timestamps on F9T-PT"},
    "fixedL1l53": {"A": "F9T-Bob", "B": "F9T-PT", "notes": "Fixed position, L1 C/A and L5 signals, run 3, longer, fixed host timestamps on F9T-PT"},
    "f9tM600-1":  {"A": "F9T-PT", "B": "M600-DHQ", "notes": "< 1 hour since boot"},
    "f9tM600-2":  {"A": "F9T-PT", "B": "M600-DHQ", "notes": "second run, stabilized"},
}

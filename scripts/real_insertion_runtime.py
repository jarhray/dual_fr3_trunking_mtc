#!/usr/bin/python3
"""Read-only by default; real insertion services require commissioning gates."""
from dual_fr3_trunking_mtc.insertion_task.real_runtime import main

if __name__ == '__main__':
    raise SystemExit(main())

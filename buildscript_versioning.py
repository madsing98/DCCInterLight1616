"""
This module stores the commit count (version / number of commits) and other version info in version.h
"""

import datetime
import subprocess

FILENAME_BUILDNO = 'versioning'
FILENAME_VERSION_H = 'include/version.h'
VERSION = 'v5.1.'

ret = subprocess.run(["git", "rev-list", "--count", "HEAD"], stdout=subprocess.PIPE, text=True)
commit_nr = ret.stdout.strip()
with open(FILENAME_BUILDNO, 'w+', encoding="utf-8") as f:
    f.write(commit_nr)
    print('Commit number: {}'.format(commit_nr))

hf = """
#ifndef COMMIT_NUMBER
  #define COMMIT_NUMBER {}
#endif
#ifndef VERSION
  #define VERSION "{} - {}"
#endif
#ifndef VERSION_SHORT
  #define VERSION_SHORT "{}"
#endif
""".format(commit_nr, VERSION+commit_nr, datetime.datetime.now(), VERSION+commit_nr)
with open(FILENAME_VERSION_H, 'w+', encoding="utf-8") as f:
    f.write(hf)

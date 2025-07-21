"""
This module stores the commit count (version / number of commits) and other version info in version.h
"""

import datetime
import subprocess

FILENAME_VERSION_H = 'include/version.h'
VERSION = 'v5.1.'

ret = subprocess.run(["git", "rev-list", "--count", "HEAD"], stdout=subprocess.PIPE, text=True)
commit_count = ret.stdout.strip()
print('[Versioning prebuild script] Git Commit Count = {}'.format(commit_count))

hf = """
#ifndef COMMIT_COUNT
  #define COMMIT_COUNT {}
#endif
#ifndef VERSION
  #define VERSION "{} - {}"
#endif
#ifndef VERSION_SHORT
  #define VERSION_SHORT "{}"
#endif
""".format(commit_count, VERSION+commit_count, datetime.datetime.now(), VERSION+commit_count)
with open(FILENAME_VERSION_H, 'w+', encoding="utf-8") as f:
    f.write(hf)

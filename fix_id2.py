import re
content = open('/home/bob/文档/备份/demo06/src/behavior_identification/src/identification.cpp', 'rb').read()
content = content.replace(b'\x00', b'')
open('/home/bob/文档/备份/demo06/src/behavior_identification/src/identification.cpp', 'wb').write(content)

import re

string="SUM FORCE: -1.3339188433518818 i : 0 b'true\r\n' SUM FORCE: -0.9396528743847891 i : 1 b'true\r\n' SUM FORCE: -1.1043448609255513 i : 2 b'true\r\n' SUM FORCE: -1.2041636448449533 i : 3 b'true\r\n' SUM FORCE: -1.1956738005257537 i : 4 b'true\r\n' SUM FORCE: -1.0219476355727177


number=re.findall(r"\d+\.?\d*",string)
print(number)

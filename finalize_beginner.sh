#!/bin/bash
# Сборка руководства новичка -> починка id картинок -> валидация -> PDF -> JPG
set -e
cd /home/claude/pvo
F="ПВО_с_нуля_для_новичка.docx"
node build_beginner.js
rm -rf /tmp/unzb && mkdir -p /tmp/unzb
unzip -q "$F" -d /tmp/unzb
python3 - <<'PY'
import re
p = '/tmp/unzb/word/document.xml'
xml = open(p, encoding='utf8').read()
c = [0]
xml = re.sub(r'wp:docPr id="1" name=""',
             lambda m: (c.__setitem__(0, c[0]+1), f'wp:docPr id="{c[0]}" name="Picture {c[0]}"')[1], xml)
c2 = [0]
xml = re.sub(r'pic:cNvPr id="0" name=""',
             lambda m: (c2.__setitem__(0, c2[0]+1), f'pic:cNvPr id="{100+c2[0]}" name="Image {c2[0]}"')[1], xml)
open(p, 'w', encoding='utf8').write(xml)
print("ids fixed:", c[0], c2[0])
PY
(cd /tmp/unzb && rm -f "/home/claude/pvo/$F" && zip -qXr "/home/claude/pvo/$F" .)
SK=$(find /root/.claude/skills -path "*docx/scripts/office/validate.py" | head -1)
python3 "$SK" "$F" | tail -1
rm -f nov-*.jpg "ПВО_с_нуля_для_новичка.pdf"
SO=$(find /root/.claude/skills -path "*docx/scripts/office/soffice.py" | head -1)
python3 "$SO" --headless --convert-to pdf "$F" >/dev/null 2>&1
pdftoppm -jpeg -r 60 "ПВО_с_нуля_для_новичка.pdf" nov
pdfinfo "ПВО_с_нуля_для_новичка.pdf" | grep Pages

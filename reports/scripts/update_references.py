#!/usr/bin/env python3
"""Render the verified bibliography; no network lookup or package installation."""
import argparse
import json
from pathlib import Path
import re


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reports', type=Path, default=Path(__file__).resolve().parents[1])
    args = ap.parse_args()
    root = args.reports.resolve()
    entries = [
        dict(bib_key='magp4_results', type='online', authors=['MAGP運営'],
             title='第4回大会 予選・決勝情報', year=2026,
             url='https://minicar-autogp.org/race/04th_results/', accessed='2026-11-15'),
        dict(bib_key='magp4_live_results', type='online', authors=['MAGP運営'],
             title='MAGP04_Q_Resuslt: Q1 results, NATC Tochigi, A16:I16', year=2026,
             url='https://docs.google.com/spreadsheets/d/1QaWHFzPd1AkKqufYSo0ngA-mUV5UsZ6hh69O2cwgjCQ/edit',
             accessed='2026-11-15'),
    ] + json.loads((root/'research/literature_sources.json').read_text())['sources']
    refs, bib = [], []
    for i, e in enumerate(entries, 1):
        authors = ' and '.join(e['authors'])
        extra = ' / '.join(str(e[k]) for k in ('edition','publisher','institution','number','version') if e.get(k))
        when = str(e['year']) if e.get('year') else '刊行・更新年未確認'
        refs.append(f"[{i}] {'; '.join(e['authors'])}. {e['title']}. {extra + '. ' if extra else ''}{when}. {e['url']} （参照 {e['accessed']}）")
        kind = {'online':'misc','software':'misc','manual':'manual','book':'book','techreport':'techreport'}[e['type']]
        fields = dict(author=authors, title=e['title'], url=e['url'], urldate=e['accessed'])
        for k in ('year','edition','publisher','institution','number','isbn','doi'):
            if e.get(k): fields[k] = str(e[k])
        if e.get('version'): fields['note'] = e['version']
        bib.append('@'+kind+'{'+e['bib_key']+',\n'+',\n'.join('  '+k+' = {'+v.replace('{','\\{').replace('}','\\}')+'}' for k,v in fields.items())+'\n}')
    (root/'references.bib').write_text('\n\n'.join(bib)+'\n')
    (root/'data/reference_index.json').write_text(json.dumps(entries, ensure_ascii=False, indent=2)+'\n')
    paper = root/'paper.md'
    text = paper.read_text()
    start = text.index('<!-- REFERENCES:GENERATED -->')
    end = text.index('\n## 付録A', start)
    text = text[:start]+'<!-- REFERENCES:GENERATED -->\n\n'+'\n\n'.join(refs)+'\n'+text[end:]
    paper.write_text(text)
    print(f'Updated {len(entries)} verified references.')


if __name__ == '__main__':
    main()

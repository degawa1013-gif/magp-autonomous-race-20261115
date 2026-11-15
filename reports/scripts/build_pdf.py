#!/usr/bin/env python3
"""Create the Japanese paper PDF with installed fonts and ReportLab."""
import argparse
import os
from datetime import datetime, timezone
from html import escape
from pathlib import Path
import re
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                TableStyle, Image, KeepTogether, HRFlowable)

FONT_JP = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
FONT_LATIN = '/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reports', type=Path, default=Path(__file__).resolve().parents[1])
    args = ap.parse_args()
    root = args.reports.resolve()
    # A deterministic report-date timestamp; do not infer the client's date from the host clock.
    os.environ.setdefault('SOURCE_DATE_EPOCH', str(int(datetime(2026,11,15,tzinfo=timezone.utc).timestamp())))
    for fn in (FONT_JP, FONT_LATIN, FONT_BOLD):
        if not Path(fn).is_file():
            raise SystemExit(f'Required existing font missing: {fn}; see environment.md. No installation performed.')
    for name, fn in [('JP',FONT_JP),('Latin',FONT_LATIN),('LatinBold',FONT_BOLD)]:
        pdfmetrics.registerFont(TTFont(name,fn))
    cmap = pdfmetrics.getFont('JP').face.charToGlyph
    palette = dict(ink=colors.HexColor('#182E3C'), gray=colors.HexColor('#53636E'))
    styles = {
        'title': ParagraphStyle('Title',fontName='JP',fontSize=19,leading=29,spaceAfter=11,textColor=palette['ink'],wordWrap='CJK'),
        'subtitle': ParagraphStyle('Subtitle',fontName='JP',fontSize=12.5,leading=21,spaceAfter=15,textColor=palette['gray'],wordWrap='CJK'),
        'h2': ParagraphStyle('Section',fontName='JP',fontSize=13.5,leading=22,spaceBefore=18,spaceAfter=8,keepWithNext=True,textColor=palette['ink'],wordWrap='CJK'),
        'h3': ParagraphStyle('Subsection',fontName='JP',fontSize=11.4,leading=19,spaceBefore=13,spaceAfter=6,keepWithNext=True,wordWrap='CJK'),
        'body': ParagraphStyle('Body',fontName='JP',fontSize=10.3,leading=17.4,spaceAfter=7,wordWrap='CJK',splitLongWords=True,allowOrphans=0,allowWidows=0),
        'small': ParagraphStyle('Small',fontName='JP',fontSize=8.5,leading=13.2,spaceAfter=7,wordWrap='CJK',splitLongWords=True),
        'caption': ParagraphStyle('Caption',fontName='JP',fontSize=9.0,leading=14.0,spaceBefore=6,spaceAfter=13,wordWrap='CJK'),
        'cell': ParagraphStyle('Cell',fontName='JP',fontSize=8.5,leading=13.0,wordWrap='CJK'),
        'code': ParagraphStyle('Equation',fontName='JP',fontSize=9.4,leading=16,leftIndent=10,spaceBefore=4,spaceAfter=8,wordWrap='CJK'),
    }
    def markup(text,bold=False):
        text = text.replace('`','').replace('**','')
        # Droid supplies CJK glyphs; DejaVu supplies Latin, Greek and math glyphs.
        chunks=[]; active=None; buf=''
        for ch in text:
            kind='JP' if ord(ch) in cmap else ('LatinBold' if bold else 'Latin')
            if kind != active and buf:
                chunks.append(f'<font name="{active}">{escape(buf)}</font>');buf=''
            active=kind;buf+=ch
        if buf: chunks.append(f'<font name="{active}">{escape(buf)}</font>')
        return ''.join(chunks)
    def para(s, kind='body', bold=False):
        return Paragraph(markup(s,bold),styles[kind])
    width = 170*mm
    story=[]; lines=(root/'paper.md').read_text().splitlines(); i=0; first_h2=True
    while i < len(lines):
        line=lines[i].strip()
        if not line or line.startswith('<!--'):
            i+=1;continue
        if line.startswith('# '):
            story.append(para(line[2:],'title',True));i+=1;continue
        if line.startswith('## '):
            kind='subtitle' if first_h2 else 'h2';first_h2=False
            story.append(para(line[3:],kind,True));i+=1;continue
        if line.startswith('### '):
            story.append(para(line[4:],'h3',True));i+=1;continue
        m=re.match(r'!\[(.*?)\]\((.*?)\)$',line)
        if m:
            p=root/m.group(2)
            if not p.is_file(): raise SystemExit(f'Missing figure: {p}')
            img=Image(str(p));height=width*img.imageHeight/img.imageWidth
            block=[Spacer(1,5),Image(str(p),width=width,height=height),para(m.group(1),'caption')]
            story.append(KeepTogether(block));i+=1;continue
        if line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].strip().startswith('|'):
                cols=[x.strip() for x in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',x) for x in cols): rows.append(cols)
                i+=1
            n=len(rows[0]); colwidths=([.22,.39,.39] if n==3 else [1/n]*n)
            cells=[[para(v,'cell',r==0) for v in row] for r,row in enumerate(rows)]
            tb=Table(cells,colWidths=[width*v for v in colwidths],repeatRows=1,hAlign='LEFT')
            tb.setStyle(TableStyle([
                ('VALIGN',(0,0),(-1,-1),'TOP'),('LINEABOVE',(0,0),(-1,0),.8,colors.black),
                ('LINEBELOW',(0,0),(-1,0),.5,colors.black),('LINEBELOW',(0,-1),(-1,-1),.8,colors.black),
                ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
                ('LEFTPADDING',(0,0),(-1,-1),5),('RIGHTPADDING',(0,0),(-1,-1),5),
            ]));story.extend([tb,Spacer(1,10)]);continue
        if lines[i].startswith('    '):
            equations=[]
            while i<len(lines) and lines[i].startswith('    '):
                equations.append(para(lines[i].strip(),'code'));i+=1
            story.append(KeepTogether(equations))
            continue
        # Source references stay in the PDF as readable, searchable text.
        is_ref=bool(re.match(r'^\[\d+\]',line))
        buf=[line];i+=1
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','|','!','    ','<!--')):
            buf.append(lines[i].strip());i+=1
        text=' '.join(buf)
        kind='small' if is_ref else 'body'
        p=para(text,kind)
        if re.match(r'^表\d',text): p.keepWithNext=True
        story.append(KeepTogether([p]) if is_ref else p)
    out=root/'output/pdf/autonomous_race_report_20261115.pdf';out.parent.mkdir(parents=True,exist_ok=True)
    def page(can,doc):
        can.saveState();can.setStrokeColor(colors.HexColor('#CED5D9'));can.setLineWidth(.4)
        can.line(20*mm,15*mm,A4[0]-20*mm,15*mm)
        can.setFont('Latin',8);can.setFillColor(palette['gray'])
        can.drawString(20*mm,10.5*mm,'MAGP 2026 / Technical report draft / 2026-11-15')
        can.drawRightString(A4[0]-20*mm,10.5*mm,str(doc.page));can.restoreState()
    doc=SimpleDocTemplate(str(out),pagesize=A4,leftMargin=20*mm,rightMargin=20*mm,
                          topMargin=18*mm,bottomMargin=22*mm,
                          title='単眼白線追従による小型自動走行車の競技開発と記録に基づく失敗解析',
                          author='NATC Tochigi - authorship details pending',pageCompression=1)
    doc.build(story,onFirstPage=page,onLaterPages=page)
    print(out)


if __name__=='__main__':main()

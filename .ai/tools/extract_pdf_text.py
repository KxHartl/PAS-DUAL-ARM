import sys
from pathlib import Path

pdf_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else pdf_path.with_suffix('.txt')

if not pdf_path or not pdf_path.exists():
    print('PDF path missing or not found')
    sys.exit(2)

text_parts = []
try:
    from pypdf import PdfReader
    reader = PdfReader(str(pdf_path))
    for p in reader.pages:
        text_parts.append(p.extract_text() or '')
except Exception:
    try:
        import PyPDF2
        reader = PyPDF2.PdfReader(str(pdf_path))
        for p in reader.pages:
            text_parts.append(p.extract_text() or '')
    except Exception as e:
        print('Failed to import pypdf/PyPDF2:', e)
        sys.exit(3)

text = '\n\n'.join(text_parts)
out_path.write_text(text, encoding='utf-8')
print('WROTE', out_path)
